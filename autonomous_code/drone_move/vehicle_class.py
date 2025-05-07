import asyncio as a
from pymavlink import mavutil
from typing import Literal
from dataclasses import dataclass
import time
import math

import builtins
import functools

print = functools.partial(builtins.print, flush=True)

@dataclass
class Vector:
        x: float
        y: float
        z: float

class Vehicle:
    VALID_MESSAGES = Literal['HEARTBEAT', 'COMMAND_ACK', 'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT',
                         'SYS_STATUS', 'MISSION_COUNT', 'MISSION_ITEM_INT', 'MISSION_CURRENT']
    VALID_MODES = Literal['GUIDED', 'LAND', 'RTL', 'AUTO', 'MANUAL', 'STABILIZE', 'LOITER']

    def __init__(self, conn='udp:localhost:14551', t=5):
        self.GPS = Vector(0, 0, 0)
        self.NED = Vector(0, 0, 0)
        self.VEL = Vector(0, 0, 0)    
        self.roll=0
        self.pitch=0    
        self.yaw = 0                             
        self.battery=100
        self.ze_connection=mavutil.mavlink_connection(conn, baud = 57600)        
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        self.prev_qual = 0   
        self.mode = 'STABILIZE'  
        self.active = True   
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.waypoint_0 = (0, 0, 0, 0) #(frame, x, y, z)
        self.t_sess = t     

        print = functools.partial(builtins.print, flush=True)

    async def track_mission_target(self):
       ##TRACK LOCATION OF DRONE AND WHERE IN SPACE TIME IT IS##
        # Request current waypoint (what the autopilot thinks is next)
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system,
            self.ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
            0, 0, 0, 0, 0, 0
        )        
        while self.active:
            if self.mode != 'AUTO':
                await a.sleep(0.5)
                continue            
            
            await a.sleep(0.1)

            # Get current waypoint index
            msg_current = await a.to_thread(self.wait_4_msg, str_type='MISSION_CURRENT')
            if not msg_current:
                print("No mission yet ':{")
                await a.sleep(1)
                continue            

            current_seq = msg_current.seq            
            await a.sleep(0.1)

            # Request the waypoint item
            self.ze_connection.mav.mission_request_int_send(
            target_system=self.ze_connection.target_system,
            target_component=self.ze_connection.target_component,
            seq=current_seq
            )

            # Get the waypoint details
            msg_wp = await a.to_thread(self.wait_4_msg, str_type='MISSION_ITEM_INT')
            if not msg_wp:
                print("No mission item retrieved! :<")
                await a.sleep(0.1)
                continue

            wp_lat = msg_wp.x / 1e7
            wp_lon = msg_wp.y / 1e7
            wp_alt = msg_wp.z / 1000                        

            # Calculate distance and bearing to the next waypoint
            distance = await a.to_thread(self.haversine(wp_lat, wp_lon))
            print(f"Next WP#{current_seq}: ({wp_lat:.7f}, {wp_lon:.7f}) | Distance: {distance:.2f} m")

            if distance > 0.1:  # Can tune this threshold
                print(f"Drone en route — verifying path...")

            else:
                print(f"[INFO] Approaching or reached waypoint #{current_seq}")

            await a.sleep(2)

    def haversine(self, lat2, lon2):
        ## CALCULATE THE SHORTEST PATH FROM LAT TO LON POINT USING HAVERSINE ##
        time.sleep(0.1)
        
        R = 6371000  # Earth radius in meters
        phi1 = math.radians(self.GPS.y)
        phi2 = math.radians(lat2)
        d_phi = math.radians(lat2 - (self.GPS.y))
        d_lambda = math.radians(lon2 - (self.GPS.x))
        a = math.sin(d_phi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))        
        return R * c    
    
    def alt_diff(self):
        ## DETERMINE IF THE CURRENT VELOCITY IS ENOUGH TO REACH WAYPOINT ALTITUDE ##
        time.sleep(0.1)
    
    async def check_telem(self):    
        ## CHECK THE TELEMETRY DATA ##
        while self.active:  
            if self.mode == 'MANUAL':
                await a.sleep(0.01)
                continue              
            t_stat, msg_stat = await a.to_thread(self.wait_4_msg, str_type="SYS_STATUS", 
                                                 time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1:                     
                    print(f"Battery info unavailable :< ")
                    self.battery=None
                else:                        
                    self.battery = msg_stat.battery_remaining  
                    await a.sleep(0.1)                    
                self.prev_qual = self.conn_qual                                         
                #self.conn_qual = msg_stat.drop_rate_comm / 10 #in %   
            else:
                if msg_stat == None:
                    self.conn_qual = 100    
            self.conn_qual = self.ze_connection.packet_loss()
            self.prev_qual = self.conn_qual            
            await a.sleep(1)     

    def wait_4_msg(self, str_type: VALID_MESSAGES, block = False, time_out_sess = None, attempts = 4):    
    ## WAIT FOR A MESSAGE FOR ONE CYCLE OR JUST UNTIL  ##
    #time_out_sess is for total time but for the rp5 its gonna be in ticks 
    #so every tick is 1 microsecond, will be spliced into attempts specificed by user or default 4   
    # 
    ## RAPS BERRY PI 5 DOES NOOOOOOOOOOOT SUPPORT PIGPIO ## 
        if time_out_sess is None:
            try:
                msg = self.ze_connection.recv_match(type = str_type, blocking = block)                                  
                return None, msg
            except Exception as e:
                return None, None
        else:                    
            #temp = time_out_sess * 1e6/ attempts
            temp = time_out_sess/ attempts
            #start_ = self.pi.get_current_tick()
            start_ = time.time()
            #while (self.pi.get_current_tick()- start_) < time_out_sess:
            while (time.time()- start_) < time_out_sess:
                try:
                    msg = self.ze_connection.recv_match(type = str_type, blocking = False)
                except Exception as e:
                    continue               
                if msg:                
                    #return (self.pi.get_current_tick() - start_), msg
                    return (time.time()- start_), msg
                time.sleep(0.1)
            print(f"No message {str_type}")
            return time_out_sess, None # when it took entire time to retrieve message and no message was retrieved      

    def update_specs(self):
        while self.active:            
            rel = 'GLOBAL_POSITION_INT'        
            # Wait for initial position
            t, msg = self.wait_4_msg(str_type=rel)
            if msg:
                self.GPS.x = msg.lat / 1e7
                self.GPS.y = msg.lon / 1e7
                self.GPS.z = msg.alt / 1000.0 
            else:
                continue
            print(f"Current Coordinate: lat = {self.GPS.x:.2f} m, lon = {self.GPS.y:.2f} m, alt = {self.GPS.z:.2f} m")    
            time.sleep(0.1)

            rel = 'LOCAL_POSITION_NED'         
            t, msg = self.wait_4_msg(str_type=rel)
            if msg:
                self.NED.x, self.NED.y, self.NED.z = msg.x, msg.y, msg.z        
            else:
                continue
            print(f"Current Position: x = {self.NED.x:.2f} m, y = {self.NED.y:.2f} m, z = {self.NED.z:.2f} m")                
            time.sleep(0.1)
            ## HOLD UNTIL POS REACHED ##  
            time.sleep(0.1)   

            rel = 'ATTITUDE'         
            t, msg = self.wait_4_msg(str_type=rel)
            if msg:
                self.roll = msg.roll * 100 / math.pi
                self.pitch = msg.pitch * 100 / math.pi
                self.yaw = msg.yaw
            else:
                continue
            print(f"Current Orientation: roll = {self.roll:.2f} m, pitch = {self.pitch:.2f} m") 

    def change_mode(self):
        ## CHANGE THE MODE OF THE DRONE ##
        mode = self.mode
        set_FC = False
        set_GCS = False
        while self.active:
            t, msg_hb = self.wait_4_msg(str_type='HEARTBEAT')
            hb_mode = None
            modes = {v: k for k, v in self.ze_connection.mode_mapping().items()}
            if msg_hb:
                print(msg_hb)
                hb_mode = msg_hb.custom_mode
            else: 
                time.sleep(0.01)
                continue
            time.sleep(0.01)
            if mode != self.mode:
                self.mode_activate(self.mode)                
            elif (hb_mode not in modes and hb_mode != self.mode):
                self.mode = modes[hb_mode]                      
            mode = self.mode      
            if mode == 'RTL' or mode == 'LAND':
                self.active = False
                time.sleep(0.01)
            print(f"Current mode: {self.mode}")
        
            time.sleep(0.01)
            if self.FC and not set_FC:
                self.ze_connection.mav.command_long_send(
                            self.ze_connection.target_system,
                            self.ze_connection.target_component,
                            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                            0,
                            0, 0, 0, 0, 0, 0, 0  # param1=0 pauses mission
                        ) 
                t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block=True)
                print(msg)                
                ## MAV_CMD_NAV_GUIDED_ENABLE 
                self.ze_connection.mav.command_long_send(
                            self.ze_connection.target_system, 
                            self.ze_connection.target_component, 
                            mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 
                            0, 1, 0, 0, 0, 0, 0, 0)    
                t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block=True)
                print(msg)
                self.mode = 'GUIDED'                 
                set_FC = True
                set_GCS = False

            elif not self.FC and not set_GCS:
                self.ze_connection.mav.command_long_send(
                            self.ze_connection.target_system,
                            self.ze_connection.target_component,
                            mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                            1,
                            1, 0, 0, 0, 0, 0, 0  # param1=1 resumes mission
                        ) 
                t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block=True)
                print(msg)            
                ## MAV_CMD_NAV_GUIDED_ENABLE 
                self.ze_connection.mav.command_long_send(
                            self.ze_connection.target_system, 
                            self.ze_connection.target_component, 
                            mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 
                            0, 0, 0, 0, 0, 0, 0, 0)    
                t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block=True)
                print(msg)                     
                set_GCS = True
                set_FC = False

    def mode_activate(self, mode_e: VALID_MODES):
        ## CAHNGE THE MODE BASED ON POSSIBLE INPUTS ##
        # Get mode ID for GUIDED
        mode_id = self.ze_connection.mode_mapping()[mode_e]
        # Send mode change request
        self.ze_connection.set_mode(mode_id)
        print(f"Mode changed to {mode_e}!") 
        time.sleep(1) 

    async def set_wrist(self, arm_disarm):
        ## SET THE DRONE TO ARMED OR DISARMED ##
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, arm_disarm, 0, 0, 0, 0, 0, 0)        
        print(a.to_thread(self.wait_4_msg(str_type='COMMAND_ACK', block = True)))
        a.sleep(0.01)  
