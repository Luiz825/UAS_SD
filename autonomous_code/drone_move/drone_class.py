import learning as ln
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import time
import os
import csv

class Drone:
    def __init__(self, t=10):
        self.ze_connection = ln.the_connection
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.active = True
        self.battery = 100
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        self.prev_qual = 0 
        self.current_waypoint_0 = (0, 0, 0, 0) #(frame, x, y, z)
        # % is the % of packages lost 
        self.mode = "STABILIZE"              
        self.t_sess = t
        self.drop = True

    async def check_telem(self):
        ## CHECK THE TELEMETRY DATA ##
        while self.active:  
            if self.mode == "MANUAL":
                continue              
            t_stat, msg_stat = await a.to_thread(self.wait_4_msg, str_type="SYS_STATUS", time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1: 
                    now = datetime.now()
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    print(f"Battery info unavailable :< {timestamp}")
                else:                        
                    self.battery = msg_stat.battery_remaining                      
                self.prev_qual = self.conn_qual                                         
                self.conn_qual = msg_stat.drop_rate_comm / 10 #in %   
            else:
                if msg_stat == None:
                    self.conn_qual = 100                
            await a.sleep(1)           

    async def grab_mission_stat(self):
        ##GRAB MISSION WAYPOINTS AND UPDATE STUFF ##
        while self.active:
            if self.mode == "MANUAL":
                continue  
            self.ze_connection.mav.mission_request_list_send(
                target_system=self.ze_connection.target_system,
                target_component=self.ze_connection.target_component
            )
            await a.sleep(0.1)
            msg_mission = await a.to_thread(self.wait_4_msg, str_type="MISSION_COUNT")
            if msg_mission and msg_mission.count > 1:                
                cnt = msg_mission.count
                print(f"Recieving #{cnt} mission waypoints!")
                for i in range(cnt):
                    self.ze_connection.mav.mission_request_int_send(
                        target_system=self.ze_connection.target_system,
                        target_component=self.ze_connection.target_component,
                        seq=i
                    )   
                    await a.sleep(0.1)
                    msg_item = None
                    print(f"Looking for waypoint {i}!")
                    while msg_item is None:
                        print(f"Search for item {i}")
                        msg_item = await a.to_thread(self.wait_4_msg, str_type = "MISSION_ITEM_INT")
                        await a.sleep(0.1)
                        if not self.active:
                            print("Died in search! :O")
                            return
                    (frame, x, y, z) = (msg_item.frame, msg_item.x, msg_item.y, msg_item.z)
                    if self.current_waypoint_0 != (frame, x, y, z) and i == 0:
                        await self.waypoint_queue.put((msg_item.frame, msg_item.x, msg_item.y, msg_item.z))
                        self.current_waypoint_0 = (frame, x, y, z)
                    else:
                        print(f"Same as previousl mission :)")
                        await a.sleep(5)
                        continue #to restart the while self.active loop in the beginning
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
                    msg_mission_start= await a.to_thread(self.wait_4_msg, "COMMAND_ACK")
                    print(msg_mission_start)
                    await a.sleep(0.1)
                self.mission = True
                print("Mission started")
            else:
                print(f"Nothing new/No new mission :<")
            await a.sleep(0.5)   

    async def mission_exec(self):
        ## EXECUTE MISSION AND WAIT UNTIL DONE ##
        current_seq = 0
        while self.active:
            if self.mode == "MANUAL":
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
                        print(f"Reached waypoint {current_seq} at {x/1e7}, {y/1e7}, {z/1000}\n at at {now.strftime("%Y-%m-%d %H:%M:%S")}")                    
                        current_seq = msg.seq
            elif self.mission and self.waypoint_queue.empty():
                self.mode = "GUIDED"
                self.mission = False
            await a.sleep(1)
    
    async def land_question(self):
        ## CHECK IF NEED TO LAND ##
        avg_qual = self.conn_qual
        iter = 0
        while self.active:   
            if self.mode == "MANUAL":
                continue           
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
    
    async def log_test(self, filename = "FILE_log.cvs", loop_time_min = 10):
        ## LOG DATA ON THE POS ##
        if not os.path.isfile(filename):
            with open(filename, mode = "w", newline = "") as file:
                scribe = csv.writer(file)
                scribe.writerow(["Timestamp", "Timelapse", "Distance"])
        
        # will only allow a time of ten minutes of recording data
        with open(filename, mode = "a", newline = "") as file:
            scribe = csv.writer(file)
            start_ = time.time()
            while ((time.time() - start_) / 60) < loop_time_min:
                now = datetime.now()                                           
                tm, msg = await a.to_thread(self.wait_4_msg("LOCAL_POSITION_NED", time_out_sess=self.t_sess, attempts=3))
                if tm == self.t_sess:
                    break
                timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
                scribe.writerow([timestamp, tm, pos])

                file.flush()
                await a.sleep(3)
    
    async def payload_sequence(self, inst):
        while self.active:
            if self.mode == "MANUAL":
                continue  
            if self.drop:
                await a.to_thread(self.move_servo, inst, 1500)
                await a.sleep(10)
                await a.to_thread(self.move_servo, inst, 1000)
                await a.sleep(2)
                self.drop = False
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
    
    async def gyro_state_awarness(self):
        while self.active:


    def wait_4_msg(self, str_type: Literal["HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED", 
                                           "HOME_POSITION", "ATTITUDE", "SYS_STATUS", "TIMESYNC", 
                                           "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CURRENT"], block = False, time_out_sess = None, attempts = 4):    
    #time_out_sess is for total time, will be spliced into attempts specificed by user or default 4
    ## WAIT FOR A MESSAGE FOR ONE CYCLE OR JUST UNTIL  ##
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
                time.sleep(0.1)
            print("No message")
            return time_out_sess, None # when it took entire time to retrieve message and no message was retrieved
        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None, xv = None, yv = None, zv = None, yaw = None):
    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete
    ## MOVE DRONE ##
        if self.mode != "GUIDED":
            self.mode = "GUIDED"
        # if all of these parameters can be organized in a list format
        # [j = getattr(str("pos.")+str(j) if j is None else j)]
        # i also don't really recall if you need to force cast thos strings

        if (x != None or y != None or z != None):
            self.waypoint_mv(frame, x, y, z, yaw)                
        
    def waypoint_mv(self, frame, x, y, z, yaw):
        ## CHANGE THE TARGET POS TO INPUT ##
        pos = self.wait_4_msg("LOCAL_POSITION_NED", block = True)
        x = pos.x if x is None else x
        y = pos.y if y is None else y
        z = 0 if z is None else z    
        yaw = self.wait_4_msg("ATTITUDE", block = True).yaw if yaw is None else yaw
        
        self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, 
                                                                                                 ln.the_connection.target_component, 
                                                                                                 mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                                                                                                 4088, x, y, (z + pos.z), 0, 0, 0, 0, 0, 0, yaw, 0))
        
        self.hold_until(frame, x, y, (z + pos.z))

    def hold_until(self, frame=1, t_x = None, t_y = None, t_z = None, tol = 0.5):
        ## HOLD UNTIL POS REACHED ##
        print("Begin to waypoint")   
        rel = "LOCAL_POSITION_NED" if frame == 1 else "GLOBAL_POSITION_INT"        
        # Wait for initial position
        pos = self.wait_4_msg(rel, block=True)
        if frame == 1:
            t_x = pos.x if t_x is None else t_x
            t_y = pos.y if t_y is None else t_y
            t_z = pos.z if t_z is None else t_z
        else:
            lat = pos.lat / 1e7
            lon = pos.lon / 1e7 
            alt = pos.alt / 1000.0  # assuming millimeters
            t_x = lat if t_x is None else t_x
            t_y = lon if t_y is None else t_y
            t_z = alt if t_z is None else t_z

        while True:
            msg = self.wait_4_msg(rel, block=True)
            if frame == 1:
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

    def mode_activate(self, mode_e: Literal["GUIDED", "LAND", "STABILIZE", "MANUAL", "LOITER", "RTL", "AUTO"]):
        ## CAHNGE THE MODE BASED ON POSSIBLE INPUTS ##
        # Get mode ID for GUIDED
        mode_id = self.ze_connection.mode_mapping()[mode_e]
        # Send mode change request
        ln.the_connection.set_mode(mode_id)
        print(f"Mode changed to {mode_e}!")   

    def set_wrist(self, arm_disarm):
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, arm_disarm, 0, 0, 0, 0, 0, 0)
        
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))    

    def settle_down(self):
        # Get mode ID for GUIDED        
        self.mode = "RTL"
        self.hold_until(t_x = 0, t_y = 0, t_z = 0, tol = 0.1) 
        self.set_wrist(0)            
        self.mode = "STABILIZE"
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

    def move_servo(self, inst, pwm):
        self.ze_connection.mav.command_long_send(
            target_system=self.ze_connection.target_system,
            target_component=self.ze_connection.target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            confirmation=0,
            param1=inst,
            param2=pwm,
            param3=0, param4=0, param5=0, param6=0, param7=0
        )  

 

                    
#trash:
# try:
    #     print(f"Going to waypoint!")
    #     # Try to get an item from the queue with a timeout (e.g., 2 seconds)
    #     frame, x, y, z = await a.wait_for(self.waypoint_queue.get(), timeout=2)

    #     # Process the waypoint
    #     #await a.to_thread(self.vel_or_waypoint_mv, frame=frame, x=x, y=y, z=z, mode=self.mode)
        
    #     # Mark this task as done
    #     self.waypoint_queue.task_done()
    #     print("Waypoint reached!")
        
    # except a.TimeoutError:
    #     # Timeout occurred, no items in the queue within 2 seconds
    #     if self.waypoint_queue.empty():
    #         self.mission = False   