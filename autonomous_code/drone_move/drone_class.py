import drone_move.learning as ln
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import time


class Drone:
    def __init__(self, t):
        self.ze_connection = ln.the_connection
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.active = True
        self.battery = 100
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        #% is the % of packages lost 
        self.mode = "STABILIZE"
        self.prev_qual = 0       
        self.t_sess = t

    async def check_telem(self):
        while self.active:                
            t_stat, msg_stat = await a.to_thread(self.wait_4_msg, str_type="SYS_STATUS", time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1: 
                    print("Battery info unavailable :<")
                else:                        
                    self.battery = msg_stat.battery_remaining                      
                self.prev_qual = self.conn_qual                                         
                self.conn_qual = msg_stat.drop_rate_comm / 10 #in %                               
            await a.sleep(1)           

    async def grab_mission_stat(self):
        while self.active:
            self.ze_connection.mav.mission_request_list_send(
                target_system=self.ze_connection.target_system,
                target_component=self.ze_connection.target_component
            )
            await a.sleep(0.01)
            msg_mission = await a.to_thread(self.wait_4_msg, str_type="MISSION_COUNT")
            if msg_mission and not self.mission and msg_mission.count > 1:
                self.mission = True
                cnt = msg_mission.count
                print(f"Recieving #{cnt} mission waypoints!")
                for i in range(cnt):
                    self.ze_connection.mav.mission_request_int_send(
                        target_system=self.ze_connection.target_system,
                        target_component=self.ze_connection.target_component,
                        seq=i
                    )
                    await a.sleep(0.01)
                    msg_item = None
                    print(f"Looking for waypoint {i}!")
                    while msg_item is None:
                        print(f"Search for item {i}")
                        msg_item = await a.to_thread(self.wait_4_msg, str_type = "MISSION_ITEM_INT")
                        await a.sleep(0.1)
                        if not self.active:
                            print("Died in search! :O")
                            return
                    await self.waypoint_queue.put(msg_item.frame, msg_item.x, msg_item.y, msg_item.z)
                    print(f"Waypoint {i} recieved!")                
                print(f"All {cnt} items recieved!")
            else:
                print(f"Nothing new/No new mission :<")
            await a.sleep(0.5)   

    async def mission_exec(self):
        while self.active:
            if self.mission and not self.waypoint_queue.empty():                    
                try:
                    print(f"Going to waypoint!")
                    # Try to get an item from the queue with a timeout (e.g., 2 seconds)
                    frame, x, y, z = await a.wait_for(self.waypoint_queue.get(), timeout=2)

                    # Process the waypoint
                    await a.to_thread(self.vel_or_waypoint_mv, frame=frame, x=x, y=y, z=z, mode=self.mode)

                    # Mark this task as done
                    self.waypoint_queue.task_done()
                    print("Waypoint reached!")
                    
                except a.TimeoutError:
                    # Timeout occurred, no items in the queue within 2 seconds
                    if self.waypoint_queue.empty():
                        self.mission = False                                    
            await a.sleep(2)
    
    async def land_question(self):
        avg_qual = self.conn_qual
        iter = 0
        while self.active:            
            avg_qual = (avg_qual + self.prev_qual + self.conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {self.battery}% at {now.strftime("%Y-%m-%d %H:%M:%S")}")
            print(f"Connection Quality: {avg_qual}% at {now.strftime("%Y-%m-%d %H:%M:%S")}")
            if (self.conn_qual > 75 and avg_qual > 75):
                iter = iter + 1                    
                if (iter == 10):
                    print(f"Comms issue!")
                    self.active = False 
            elif self.battery < 50:
                self.active = False
                print(f"Battery low!")
            else:
                iter = 0            
            await a.sleep(1)
        print(f"Issue occured!")
        self.mission = False
        await a.to_thread(self.settle_down)
        await a.sleep(2)

    def wait_4_msg(self, str_type: Literal["HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED", "HOME_POSITION", "ATTITUDE", "SYS_STATUS", "TIMESYNC", "MISSION_COUNT", "MISSION_ITEM_INT"], block = False, time_out_sess = None, attempts = 4):    
    #time_out_sess is for total time, will be spliced into attempts specificed by user or default 4
        if time_out_sess is None:
            msg = self.ze_connection.recv_match(type = str_type, blocking = block)
            return msg
        else:        
            temp = time_out_sess / attempts
            start_ = time.time()        
            while (time.time() - start_) < time_out_sess:
                msg = self.ze_connection.recv_match(type = str_type, blocking = False, timeout = temp)
                if msg is None:
                    continue
                elif msg:                
                    return (time.time() - start_), msg
                time.sleep(0.01)
            print("No message")
            return time_out_sess, None # when it took entire time to retrieve message and no message was retrieved
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None, xv = None, yv = None, zv = None, yaw = None, mode = "GUIDED"):
    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete
        if mode != "GUIDED":
            self.mode_activate("GUIDED")
        # if all of these parameters can be organized in a list format
        # [j = getattr(str("pos.")+str(j) if j is None else j)]
        # i also don't really recall if you need to force cast thos strings

        if (x != None or y != None or z != None):
            self.waypoint_mv(frame, x, y, z, yaw)                
        
    def waypoint_mv(self, frame, x, y, z, yaw):
        pos = self.wait_4_msg("LOCAL_POSITION_NED", block = True)
        x = pos.x if x is None else x
        y = pos.y if y is None else y
        z = 0 if z is None else z    
        yaw = self.wait_4_msg("ATTITUDE", block = True).yaw if yaw is None else yaw
        
        ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 4088, x, y, (z + pos.z), 0, 0, 0, 0, 0, 0, yaw, 0))
        self.hold_until(frame, x, y, (z + pos.z))
    def hold_until(self, frame=1, t_x = None, t_y = None, t_z = None, tol = 0.5):
        print("Begin to waypoint")   
        rel = "LOCAL_POSITION_NED" if frame != 0 else "GLOBAL_POSITION_INT"
        
        # Wait for initial position
        pos = self.wait_4_msg(rel, block=True)
        if rel == "LOCAL_POSITION_NED":
            t_x = pos.x if t_x is None else t_x
            t_y = pos.y if t_y is None else t_y
            t_z = pos.z if t_z is None else t_z
        else:
            lat = pos.lat / 1e7
            lon = pos.lon / 1e7 
            alt = pos.alt / 1000.0  # assuming millimeters
            t_x = lat if t_x is None else t_x / 1e7
            t_y = lon if t_y is None else t_y / 1e7
            t_z = alt if t_z is None else t_z /1e7

        while True:
            msg = self.wait_4_msg(rel, block=True)
            if rel == "LOCAL_POSITION_NED":
                x, y, z = msg.x, msg.y, msg.z
            else:
                x = msg.lat / 1e7
                y = msg.lon / 1e7
                z = msg.alt / 1000.0 
            print(f"Current Position: x = {x:.2f} m, y = {y:.2f} m, z = {z:.2f} m")
            print(f"Target Position: x = {t_x:.2f} m, y = {t_y:.2f} m, z = {t_z:.2f} m")
            if(abs(t_x - x) < tol and abs(t_y - y) < tol and abs(t_z - z) < tol):
                print("Position set")  
                return  

    def mode_activate(self, mode_e: Literal["GUIDED", "LAND", "STABILIZE", "MANUAL", "LOITER", "RTL"]):
        # Get mode ID for GUIDED
        mode_id = self.ze_connection.mode_mapping()[mode_e]
        # Send mode change request
        ln.the_connection.set_mode(mode_id)
        print(f"Mode changed to {mode_e}!")   

    def guide(self):
        # MAV_CMD_NAV_GUIDED_ENABLE 
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 
            0, 1, 0, 0, 0, 0, 0, 0)

    def set_wrist(self, arm_disarm):
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, arm_disarm, 0, 0, 0, 0, 0, 0)
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))    

    def manual_sett(self):
        self.ze_connection.mav.manual_control_send(self.ze_connection.target_system, 0, 0, 500,  0,  0)
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True)) 

    def settle_down(self):
        # Get mode ID for GUIDED
        self.mode_activate("RTL") 
        self.hold_until(t_z = 0, tol = 1)    
        self.set_wrist(0)    
        self.mode_activate("STABILIZE")
        print(self.wait_4_msg(str_type="HEARTBEAT", block=True))
        
    def to_infinity_and_beyond(self, h, yaw = 0):     
        self.mode_activate("GUIDED")
        self.set_wrist(1)
        self.the_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, yaw, 0, 0, h)    
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))
        self.hold_until(t_z = abs(h) * -1)     

 

                    
