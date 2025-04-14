import vehicle_class as vc
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import time
import pigpio
import os
import csv

class Drone(vc.Vehicle):
    VALID_MESSAGES = vc.Vehicle.VALID_MESSAGES + Literal[""]
    VALID_MODES = vc.Vehicle.VALID_MODES + Literal[ "STABILIZE", "LOITER",]

    def __init__(self, conn, t=10):
        super().__init__(conn, t)
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.active = True                
        self.current_waypoint_0 = (0, 0, 0, 0) #(frame, x, y, z)
        # % is the % of packages lost 
        self.mode = "STABILIZE"              
        self.t_sess = t
        self.drop = False
        self.pi = pigpio.pi()

    async def grab_mission_stat(self):
        ##GRAB MISSION WAYPOINTS AND UPDATE STUFF ##
        while self.active:
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue  
            super().ze_connection.mav.mission_request_list_send(
                target_system=super().ze_connection.target_system,
                target_component=super().ze_connection.target_component
            )
            await a.sleep(0.1)
            msg_mission = await a.to_thread(super().wait_4_msg, str_type="MISSION_COUNT")
            if msg_mission and msg_mission.count > 1:                
                cnt = msg_mission.count
                print(f"Recieving #{cnt} mission waypoints!")
                for i in range(cnt):
                    super().ze_connection.mav.mission_request_int_send(
                        target_system=super().ze_connection.target_system,
                        target_component=super().ze_connection.target_component,
                        seq=i
                    )   
                    await a.sleep(0.1)
                    msg_item = None
                    print(f"Looking for waypoint {i}!")
                    while msg_item is None:
                        print(f"Search for item {i}")
                        msg_item = await a.to_thread(super().wait_4_msg, str_type = "MISSION_ITEM_INT")
                        await a.sleep(0.1)
                        if not self.active:
                            print("Died in search! :O")
                            return
                    (frame, x, y, z) = (msg_item.frame, msg_item.x, msg_item.y, msg_item.z)
                    if self.current_waypoint_0 != (frame, x, y, z) and i == 0:
                        await self.waypoint_queue.put((msg_item.frame, msg_item.x, msg_item.y, msg_item.z))
                        self.current_waypoint_0 = (frame, x, y, z)
                    else:
                        print(f"Same as previous mission :)")
                        await a.sleep(2)
                        continue #to restart the while self.active loop in the beginning
                    print(f"Waypoint {i} recieved! {msg_item.x/1e7}, {msg_item.y/1e7}, {msg_item.z/1000}")                
                print(f"All {cnt} items recieved!")                
                self.mode = "AUTO"
                super().ze_connection.mav.command_long_send(
                    super().ze_connection.target_system,
                    super().ze_connection.target_component,
                    mavutil.mavlink.MAV_CMD_MISSION_START,
                    0, 0, 0, 0, 0, 0, 0, 0)
                msg_mission_start = None
                while msg_mission_start is None:
                    msg_mission_start= await a.to_thread(super().wait_4_msg, "COMMAND_ACK")
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
                await a.sleep(0.01)
                continue   
            print(f"Mission execution: {current_seq}")
            now = datetime.now()
            if self.mission and not self.waypoint_queue.empty(): 
                while self.mode != "AUTO":
                    await a.sleep(0.5)
                msg = await a.to_thread(super().wait_4_msg, str_type="MISSION_CURRENT")
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
    
    async def land_question(self):
        ## CHECK IF NEED TO LAND ##
        avg_qual = super().conn_qual
        iter = 0
        while self.active:   
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue           
            avg_qual = (avg_qual + super().prev_qual + super().conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {super().battery}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")           
            print(f"Connection Quality: {avg_qual}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")            
            if (super().conn_qual > 40 and avg_qual > 40):
                iter = iter + 1                    
                if (iter == 10):
                    print(f"Comms issue!")
                    self.active = False 
            elif super().battery < 50:
                self.active = False
                print(f"Battery low!")
            else:
                iter = 0            
            await a.sleep(1)
        print(f"Issue occured!")
        self.mission = False
        await a.to_thread(self.settle_down)
        await a.sleep(2)
    
    async def log_test_pigpio_ed(self, filename = "FILE_log.cvs", loop_time_min = 10, conn=True):
        ## LOG DATA ON THE POS ##
        # the loop time is in minutes
        if not os.path.isfile(filename):
            with open(filename, mode = "w", newline = "") as file:
                scribe = csv.writer(file)                
                if not conn:
                    scribe.writerow(["Timestamp", "Timelapse", "Distance"]) 
                else:
                    scribe.writerow(["Timestamp", "Timelapse", "Data"]) 
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system,
            super().ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,  # message id for RAW_IMU
            20000,  # 20,000 µs = 50 Hz
            0, 0, 0, 0, 0
        )
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system,
            super().ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,  # <- Changed to SYS_STATUS
            22000,  # Interval in microseconds (22,000 µs = ~45.45 Hz)
            0, 0, 0, 0, 0
        )
        # will only allow a time of ten minutes (default) of recording data
        with open(filename, mode = "a", newline = "") as file:
            scribe = csv.writer(file)
            start_ = self.pi.get_current_tick()
            while (((self.pi.get_current_tick() - start_) * 1e6)/60) < loop_time_min:
                now = datetime.now()             
                if not conn:
                    tm, msg = await a.to_thread(super().wait_4_msg,"LOCAL_POSITION_NED", 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
                    scribe.writerow([timestamp, tm, pos])
                else:
                    tm, msg = await a.to_thread(super().wait_4_msg,"RAW_IMU", 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        print(f"Not talking @ {timestamp}!")
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")                    
                    scribe.writerow([timestamp, tm, msg])   
                file.flush()
                await a.sleep(3)

    async def log_test_time_ed(self, filename = "FILE_log.cvs", loop_time_min = 10, conn=True):
        ## LOG DATA ON THE POS ##
        # the loop time is in minutes
        if not os.path.isfile(filename):
            with open(filename, mode = "w", newline = "") as file:
                scribe = csv.writer(file)                
                if not conn:
                    scribe.writerow(["Timestamp", "Timelapse", "Distance"]) 
                else:
                    scribe.writerow(["Timestamp", "Timelapse", "Data", "Health"]) 
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system,
            super().ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,  # message id for RAW_IMU
            20000,  # 20,000 µs = 50 Hz
            0, 0, 0, 0, 0
        )
        
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system,
            super().ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,  # <- Changed to SYS_STATUS
            22000,  # Interval in microseconds (22,000 µs = ~45.45 Hz)
            0, 0, 0, 0, 0
        )

        # will only allow a time of ten minutes (default) of recording data
        with open(filename, mode = "a", newline = "") as file:
            scribe = csv.writer(file)
            start_ = time.time()
            while ((time.time() - start_)/60) < loop_time_min:
                now = datetime.now()    
                timestamp = now.strftime("%Y/%m/%d %H:%M:%S")          
                if not conn:
                    tm, msg = await a.to_thread(super().wait_4_msg,"LOCAL_POSITION_NED", 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
                    scribe.writerow([timestamp, tm, pos])
                else:
                    tm_raw, msg_raw = await a.to_thread(super().wait_4_msg,"RAW_IMU", 
                                                        time_out_sess=self.t_sess, attempts=3)
                    if tm_raw == self.t_sess:
                        print(f"Not talking @ {timestamp}!")
                        break
                    tm, msg = await a.to_thread(super().wait_4_msg,"SYS_STATUS", 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        print(f"Not talking @ {timestamp}!")
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")                    
                    scribe.writerow([timestamp, tm, msg_raw, msg.drop_rate_comm])   
                file.flush()
                await a.sleep(3)
    
    async def payload_sequence(self, inst):
        ## SPECIFIC SEQUENCE OF VALUES FOR PAYLOAD DROP ##
        while self.active:
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue  
            if self.drop:
                await a.to_thread(self.move_servo, inst, 1500)
                await a.sleep(10)
                await a.to_thread(self.move_servo, inst, 1000)
                await a.sleep(2)
                self.drop = False
            await a.sleep(0.1)

    
    
    # async def gyro_state_awarness(self):
    #     while self.active:    
        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None, 
                           xv = None, yv = None, zv = None, yaw = None):
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
        pos = super().wait_4_msg("LOCAL_POSITION_NED", block = True)
        x = pos.x if x is None else x
        y = pos.y if y is None else y
        z = 0 if z is None else z    
        yaw = super().wait_4_msg("ATTITUDE", block = True).yaw if yaw is None else yaw
        
        super().ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, super().ze_connectionthe_connection.target_system, 
            super().ze_connection.the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            4088, x, y, (z + pos.z), 0, 0, 0, 0, 0, 0, yaw, 0))             

    def set_wrist(self, arm_disarm):
        ## SET THE DRONE TO ARMED OR DISARMED ##
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system, 
            super().ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 
            0, arm_disarm, 0, 0, 0, 0, 0, 0)        
        print(super().wait_4_msg(str_type="COMMAND_ACK", block = True))  

      

    def settle_down(self, tol=0.05):
        ## SETT DRONE BACK TO LAND ##
        # tolerance default is 50mm
        self.mode = "RTL"
        target_x, target_y, target_z = 0, 0, 0
        while not abs(super().x - target_x) < tol and abs(super().y - target_y) < tol and abs(super().z - target_z) < tol:
            #tolerance same for all         
            if self.mode != "LAND" and super().battery < 10:
                self.mode = "LAND"
                target_x, target_y = super().x, super().y
        self.set_wrist(0)
        self.mode = "STABILIZE"
        print(super().wait_4_msg(str_type="HEARTBEAT", block=True))
        
    def to_infinity_and_beyond(self, h, yaw = 0):   
        ## TAKE OFF AND REACH AN ALTITUDE FOR GUIDED MODE/WHEN STARTING FOR  ##  
        self.mode = "GUIDED"
        self.set_wrist(1)
        super().ze_connection.mav.command_long_send(
            super().ze_connection.target_system, 
            super().ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, yaw, 0, 0, h)    
        print(super().wait_4_msg(str_type="COMMAND_ACK", block = True))          

    def move_servo(self, inst, pwm):
        ## MOVE THE MOTOR AT INST A VAL OF PWM ##
        super().ze_connection.mav.command_long_send(
            target_system=super().ze_connection.target_system,
            target_component=super().ze_connection.target_component,
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