import vehicle_class as vc
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import math
import time
# import pigpio
import os
import csv
import math

class Drone(vc.Vehicle):
    VALID_MESSAGES = vc.Vehicle.VALID_MESSAGES + Literal[""]
    VALID_MODES = vc.Vehicle.VALID_MODES + Literal[ "STABILIZE", "LOITER",]

    def __init__(self, conn, t=10):
        super().__init__(conn, t)        
        self.roll=0
        self.pitch=0                                
        # % is the % of packages lost                 
        self.drop = False        
    
    async def land_question(self):
        ## CHECK IF NEED TO LAND ##
        avg_qual = super().conn_qual  
        start_ = 0      
        while super().active:   
            if super().mode == "MANUAL":
                await a.sleep(0.01)
                continue           
            avg_qual = (avg_qual + super().prev_qual + super().conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {super().battery}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")           
            print(f"Connection Quality: {avg_qual}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")            
            if (super().conn_qual > 40 and avg_qual > 40):
                if start_ == 0:
                    #start_ = super().pi.get_current_tick()
                    start_ = time.time()
                #elif (super().pi.get_current_tick() - start_) > 2500:
                elif (time.time() - start_) > 25:
                    print(f"Comms issue!")
                    super().active = False 
                    start_ = 0
                else:
                    start_ = 0
            elif super().battery < 20:
                super().active = False
                print(f"Battery low!")                    
            await a.sleep(1)
        print(f"Issue occured!")
        self.mission = False
        await a.to_thread(self.settle_down)
        await a.sleep(2)
    
    async def log_test(self, filename = "FILE_log.cvs", loop_time_min = 10, conn=True):
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
            #start_ = super().pi.get_current_tick()
            start_ = time.time()
            #while (((super().pi.get_current_tick() - start_) * 1e6)/60) < loop_time_min:
            while ((time.time() - start_)/60) < loop_time_min:
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
    
    async def payload_sequence(self, inst):
        ## SPECIFIC SEQUENCE OF VALUES FOR PAYLOAD DROP ##
        while super().active:
            if super().mode == "MANUAL":
                await a.sleep(0.01)
                continue  
            if self.drop:
                await a.to_thread(self.move_servo, inst, 1500)
                await a.sleep(10)
                await a.to_thread(self.move_servo, inst, 1000)
                await a.sleep(2)
                self.drop = False
            await a.sleep(0.1)

    async def crash_check(self, tol = 0.5):
        ## IF THE DRONE SHIFTS EXTREME TO ANGLE GRATER 100D THEN STOP TO LAND##
        # crashing land tolerance of 1/2 meter #
        start_ = 0
        while super().active:
            if self.roll > 70 or self.pitch > 70:
                if start_ is 0:
                    #start_ = super().pi.get_current_tick()
                    start_ = time.time()
                #elif (super().pi.get_current_tick() - start_) > 300:
                elif (time.time() - start_) > 2:
                    self.active = False
                    super().mode = "LAND"
                    start_ = 0
                else:
                    start_ = 0
            await a.sleep(0.1)
        while (super().y - tol) > tol:
            continue

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0,  # Confirmation
            1,  # Termination ON
            0, 0, 0, 0, 0, 0
        )
    
    async def update_GYRO(self):
        while super().active:    
            ## HOLD UNTIL POS REACHED ##  
            await a.sleep(0.1)         
            rel = "ATTITUDE"         
            msg = await a.to_thread(self.wait_4_msg, str_type=rel)
            if msg:
                self.roll = msg.roll * 100 / math.pi
                self.pitch = msg.pitch * 100 / math.pi
            print(f"Current Orientation: roll = {self.roll:.2f} m, pitch = {self.pitch:.2f} m") 
        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None, 
                           xv = None, yv = None, zv = None, yaw = None):
    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete
    ## MOVE DRONE ##
        if super().mode != "GUIDED":
            super().mode = "GUIDED"
        # if all of these parameters can be organized in a list format
        # [j = getattr(str("pos.")+str(j) if j is None else j)]
        # i also don't really recall if you need to force cast thos strings

        if (x != None or y != None or z != None):
            self.waypoint_mv(frame, x, y, z, yaw)                
        
    def waypoint_mv(self, frame=1, x=0, y=0, z=0, yaw=0):
        ## CHANGE THE TARGET POS TO INPUT ##               
        super().ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            0, super().ze_connectionthe_connection.target_system, 
            super().ze_connection.the_connection.target_component, 
            mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
            4088, 
            (x + super().x) if frame == 1 else (x + super().lon), 
            (y + super().y) if frame == 1 else (y + super().lat), 
            (z + abs(super().z)) if frame == 1 else (z + super().alt), 
            0, 0, 0, 0, 0, 0, yaw, 0))     
    print(super().wait_4_msg(str_type="COMMAND_ACK", block = True))                       

    def settle_down(self, tol=0.05):
        ## SETT DRONE BACK TO LAND ##
        # tolerance default is 50mm
        super().mode = "RTL"
        target_x, target_y, target_z = 0, 0, 0
        while not abs(super().x - target_x) < tol and abs(super().y - target_y) < tol and abs(super().z - target_z) < tol:
            #tolerance same for all         
            if super().mode != "LAND" and super().battery < 10:
                super().mode = "LAND"
                target_x, target_y = super().x, super().y
        self.set_wrist(0)
        super().mode = "STABILIZE"
        print(super().wait_4_msg(str_type="HEARTBEAT", block=True))
        
    def to_infinity_and_beyond(self, h=0.25, yaw = 0):   
        ## TAKE OFF AND REACH AN ALTITUDE FOR GUIDED MODE/WHEN STARTING FOR  ##  
        super().mode = "GUIDED"
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