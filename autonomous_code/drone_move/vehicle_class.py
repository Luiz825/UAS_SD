import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import pigpio
import time

class Vehicle:
    VALID_MESSAGES = Literal["HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT",
                          "SYS_STATUS", "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CURRENT"]
    VALID_MODES = Literal["GUIDED", "LAND", "RTL", "AUTO", "MANUAL"]

    def __init__(self, conn='udp:localhost:14551', t=5):
        self.x=0
        self.y=0
        self.z=0    
        self.lat=0
        self.lon=0
        self.alt=0          
        self.battery=100
        self.ze_connection=mavutil.mavlink_connection(conn, baud = 57600)
        self.pi = pigpio.pi()
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        self.prev_qual = 0   
        self.mode = "STABILIZE"   
        self.active = True   
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.waypoint_0 = (0, 0, 0, 0) #(frame, x, y, z)
        self.t_sess = t
    
    async def grab_mission_stat(self):
        ##GRAB MISSION WAYPOINTS AND UPDATE STUFF ##
        while self.active:
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue  
            self.ze_connection.mav.mission_request_list_send(
                target_system=self.ze_connection.target_system,
                target_component=self.ze_connection.target_component
            )
            await a.sleep(0.1)
            msg_mission = await a.to_thread(self.wait_4_msg, str_type="MISSION_COUNT")
            if msg_mission and msg_mission.count > 1:                
                cnt = msg_mission.count
                msg_item = await self._grab_waypoint(0)
                if self.waypoint_0 != msg_item:
                    await self.waypoint_queue.put((msg_item.frame, msg_item.x, msg_item.y, msg_item.z))
                    self.waypoint_0 = msg_item
                else:
                    print(f"Same as previous mission :)")
                    await a.sleep(2)
                    continue #to restart the while self.active loop in the beginning
                print(f"Recieving #{cnt} mission waypoints!")
                for i in range(1, cnt):
                    msg_item = await self._grab_waypoint(i)                    
                    await self.waypoint_queue.put((msg_item.frame, msg_item.x, msg_item.y, msg_item.z))
                    print(f"Waypoint {i} recieved! {msg_item.x/1e7}, {msg_item.y/1e7}, {msg_item.z/1000}")                
                print(f"All {cnt} items recieved!")                
                self.mode = "AUTO"
                self.ze_connection.mav.command_long_send(
                    self.ze_connection.target_system,
                    self.ze_connection.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START,
                    0, 0, 0, 0, 0, 0, 0, 0)
                msg_mission_start = None
                while msg_mission_start is None:
                    msg_mission_start= await a.to_thread(self.wait_4_msg, "MISSION_ACK")
                    print(msg_mission_start)
                    await a.sleep(0.1)
                self.mission = True
                print("Mission started")
            else:
                print(f"Nothing new/No new mission :<")
            await a.sleep(0.5)   
    
    async def _grab_waypoint(self, seq):
        self.ze_connection.mav.mission_request_int_send(
            target_system=self.ze_connection.target_system,
            target_component=self.ze_connection.target_component,
            seq=seq
        )   
        await a.sleep(0.1)
        msg_item = None
        print(f"Looking for waypoint {seq}!")
        while msg_item is None:
            print(f"Search for item {seq}")
            msg_item = await a.to_thread(self.wait_4_msg, str_type = "MISSION_ITEM_INT")
            await a.sleep(0.1)
            if not self.active:
                print("Died in search! :O")
                return
        wp = (msg_item.frame, msg_item.x, msg_item.y, msg_item.z)                    
        return wp

    async def mission_exec(self):
        ## EXECUTE MISSION AND WAIT UNTIL DONE ##
        current_seq = 0
        while self.active:   
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue   
            print(f"Mission execution: {current_seq}")
            now = datetime.now()
            if self.mission and not self.waypoint_queue.empty(): 
                while self.mode != "AUTO":
                    await a.sleep(0.5)
                msg = await a.to_thread(self.wait_4_msg, str_type="MISSION_CURRENT")
                if msg:
                    if msg.seq == current_seq + 1:
                        frame, x, y, z = await self.waypoint_queue.get()
                        print(f"Reached waypoint {current_seq} at {x/1e7}, {y/1e7}, \
                              {z/1000} at {now.strftime('%Y-%m-%d %H:%M:%S')}")                    
                        current_seq = msg.seq
            elif self.mission and self.waypoint_queue.empty():
                self.mode = "GUIDED"
                self.mission = False
            await a.sleep(1)
    
    async def check_telem(self):    
        ## CHECK THE TELEMETRY DATA ##
        while self.active:  
            if self.mode == "MANUAL":
                continue              
            t_stat, msg_stat = await a.to_thread(self.wait_4_msg, str_type="SYS_STATUS", 
                                                 time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1:                     
                    print(f"Battery info unavailable :< ")
                    self.battery=None
                else:                        
                    self.battery = msg_stat.battery_remaining                      
                self.prev_qual = self.conn_qual                                         
                self.conn_qual = msg_stat.drop_rate_comm / 10 #in %   
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
        if time_out_sess is None:
            msg = self.ze_connection.recv_match(type = str_type, blocking = block)
            return msg
        else:                    
            temp = time_out_sess * 1e6/ attempts
            start_ = self.pi.get_current_tick()
            while (self.pi.get_current_tick()- start_) < time_out_sess:
                msg = self.ze_connection.recv_match(type = str_type, blocking = False, timeout = temp)
                if msg is None:
                    continue
                elif msg:                
                    return (self.pi.get_current_tick() - start_), msg
                time.sleep(0.1)
            print("No message")
            return time_out_sess, None # when it took entire time to retrieve message and no message was retrieved      

    async def update_NED(self):
        ## HOLD UNTIL POS REACHED ##   
        while self.active:        
            rel = "LOCAL_POSITION_NED"         
            msg = await a.to_thread(self.wait_4_msg, str_type=rel)
            if msg:
                self.x, self.y, self.z = msg.x, msg.y, msg.z        
            print(f"Current Position: x = {self.x:.2f} m, y = {self.y:.2f} m, z = {self.z:.2f} m")                
            await a.sleep(0.1)
        
    async def update_GPS(self):
        ## HOLD UNTIL POS REACHED ##     
        while self.active:            
            rel = "GLOBAL_POSITION_INT"        
            # Wait for initial position
            msg = await a.to_thread(self.wait_4_msg, str_type=rel)
            if msg:
                self.lat = msg.lat / 1e7
                self.lon = msg.lon / 1e7
                self.alt = msg.alt / 1000.0 
            print(f"Current Coordinate: lat = {self.lat:.2f} m, lon = {self.lon:.2f} m, alt = {self.alt:.2f} m")    
            await a.sleep(0.1)

    async def change_mode(self):
        ## CHANGE THE MODE OF THE DRONE ##
        mode = self.mode
        while True:
            msg_hb = await a.to_thread(self.wait_4_msg, str_type="HEARTBEAT")
            hb_mode = None
            if msg_hb:
                hb_mode = msg_hb.custom_mode
            await a.sleep(0.1)
            if mode != self.mode or (hb_mode != None and hb_mode != self.mode):
                self.mode_activate(self.mode)
                mode = self.mode
            await a.sleep(0.1) 

    def mode_activate(self, mode_e: VALID_MODES):
        ## CAHNGE THE MODE BASED ON POSSIBLE INPUTS ##
        # Get mode ID for GUIDED
        mode_id = self.ze_connection.mode_mapping()[mode_e]
        # Send mode change request
        self.ze_connection.set_mode(mode_id)
        print(f"Mode changed to {mode_e}!") 

    def set_wrist(self, arm_disarm):
        ## SET THE DRONE TO ARMED OR DISARMED ##
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, arm_disarm, 0, 0, 0, 0, 0, 0)        
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True)) 