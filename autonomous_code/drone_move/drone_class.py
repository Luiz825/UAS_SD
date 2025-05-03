import vehicle_class as vc
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import math
import time
import os
import csv
from collections import deque #MAKE INTO ASYNC QUEUE
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as nP
import hailo
import sys
from collections import deque #MAKE INTO ASYNC QUEUE
from camera_class import user_app_callback_class
import builtins
import functools

print = functools.partial(builtins.print, flush=True)

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

class Drone(vc.Vehicle):
    #VALID_MESSAGES = vc.Vehicle.VALID_MESSAGES + Literal[""]
    #VALID_MODES = vc.Vehicle.VALID_MODES + Literal[ "STABILIZE", "LOITER"]

    def __init__(self, conn, t=10):
        super(Drone, self).__init__(conn=conn, t=t)        
        self.roll=0
        self.pitch=0    
        self.yaw = 0                            
        # % is the % of packages lost                         
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system,
            self.ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,  # message id for RAW_IMU
            100000,  # 100,000 µs = 10Hz
            0, 0, 0, 0, 0
        )
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system,
            self.ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # message id for RAW_IMU
            100000,  # 100,000 µs = 10Hz
            0, 0, 0, 0, 0
        )
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system,
            self.ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,  # message id for RAW_IMU
            20000,  # 20,000 µs = 50 Hz
            0, 0, 0, 0, 0
        )
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system,
            self.ze_connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,  # <- Changed to SYS_STATUS
            22000,  # Interval in microseconds (22,000 µs = ~45.45 Hz)
            0, 0, 0, 0, 0
        )
    
    async def land_question(self):
        ## CHECK IF NEED TO LAND ##

        avg_qual = self.conn_qual  
        start_ = 0      
        while self.active:   
            if self.mode == 'MANUAL':
                await a.sleep(0.01)
                continue           
            avg_qual = (avg_qual + self.prev_qual + self.conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {self.battery}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")           
            print(f"Connection Quality: {avg_qual}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")            
            if (self.conn_qual > 80 and avg_qual > 80):
                if start_ == 0:
                    #start_ = self.pi.get_current_tick()
                    start_ = time.time()
                #elif (self.pi.get_current_tick() - start_) > 2500:
                elif (time.time() - start_) > 25:
                    print(f"Comms issue!")
                    self.set_FComp()
                    self.mode = 'RTL'
                    self.active = False 
                    start_ = 0
                else:
                    start_ = 0
            elif self.battery is not None and self.battery < 20: 
                #fix the battery monitor on the flight controller
                self.set_FComp()
                self.active = False
                print(f"Battery low!")                    
            await a.sleep(1)
        print(f"No longer active, ending processes")
        self.mission = False
        await a.to_thread(self.settle_down)
        await a.sleep(2)
    
    async def log_test(self, filename = "FILE_log.csv", loop_time_min = 10, conn=True):
        ## LOG DATA ON THE POS ##

        # the loop time is in minutes
        if not os.path.isfile(filename):
            with open(filename, mode = "w", newline = "") as file:
                scribe = csv.writer(file)                
                if not conn:
                    scribe.writerow(["Timestamp", "Timelapse", "Distance"]) 
                else:
                    scribe.writerow(["Timestamp", "Timelapse", "Data"])         
        # will only allow a time of ten minutes (default) of recording data
        with open(filename, mode = "a", newline = "") as file:
            scribe = csv.writer(file)
            #start_ = self.pi.get_current_tick()
            start_ = time.time()
            #while (((self.pi.get_current_tick() - start_) * 1e6)/60) < loop_time_min:
            while ((time.time() - start_)/60) < loop_time_min:
                now = datetime.now()             
                if not conn:
                    tm, msg = await a.to_thread(self.wait_4_msg,'LOCAL_POSITION_GPS', 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
                    scribe.writerow([timestamp, tm, pos])
                else:
                    tm, msg = await a.to_thread(self.wait_4_msg,'RAW_IMU', 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        print(f"Not talking @ {timestamp}!")
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")                    
                    scribe.writerow([timestamp, tm, msg])   
                file.flush()
                await a.sleep(3)
    
    def payload_sequence(self, inst=8):
        ## SPECIFIC SEQUENCE OF VALUES FOR PAYLOAD DROP ##

        while self.active:
            if self.mode == 'MANUAL':
                time.sleep(0.01)
                continue  
            if self.drop:
                self.move_servo(inst, 850)
                time.sleep(0.01)
                self.move_servo(inst, 1550)
                time.sleep(0.01)
                self.drop = False
            time.sleep(0.01)

    async def crash_check(self, tol = 0.5):
        ## IF THE DRONE SHIFTS EXTREME TO ANGLE GRATER 100D THEN STOP TO LAND ##

        # crashing land tolerance of 1/2 meter #
        start_ = 0
        while self.active:
            if self.roll > 70 or self.pitch > 70:
                if start_ == 0:
                    #start_ = self.pi.get_current_tick()
                    start_ = time.time()
                #elif (self.pi.get_current_tick() - start_) > 300:
                elif (time.time() - start_) > 2:
                    self.set_FComp()
                    self.active = False
                    self.mode = 'LAND'
                    start_ = 0
                else:
                    start_ = 0
            await a.sleep(0.1)
        while (self.NED.y - tol) > tol:
            await a.sleep(0.1)
            continue
        self.set_wrist(0)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0,  # Confirmation
            1,  # Termination ON
            0, 0, 0, 0, 0, 0)
    
    async def update_GYRO(self):
        ## UPDATE GRYO SPEC OF THE DRONE CONTINOUSLY ##

        while self.active:    
            ## HOLD UNTIL POS REACHED ##  
            await a.sleep(0.1)         
            rel = 'ATTITUDE'         
            t, msg = await a.to_thread(self.wait_4_msg, str_type=rel)
            if msg:
                self.roll = msg.roll * 100 / math.pi
                self.pitch = msg.pitch * 100 / math.pi
                self.yaw = msg.yaw
            else:
                continue
            print(f"Current Orientation: roll = {self.roll:.2f} m, pitch = {self.pitch:.2f} m") 
        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None):
    ## MOVE DRONE ##

    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete        
        self.set_FComp()        
        if self.mode != 'GUIDED':
            self.mode = 'GUIDED'
        # if all of these parameters can be organized in a list format
        # [j = getattr(str("pos.")+str(j) if j is None else j)]
        # i also don't really recall if you need to force cast thos strings
        time.sleep(1)
        if (x != None or y != None or z != None):
            x = 0.0 if x is None else x
            y = 0.0 if y is None else y
            z = 0.0 if z is None else z
            self.waypoint_mv(frame, x, y, z)
        
    def waypoint_mv(self, frame=1, x=0, y=0, z=0):
        ## CHANGE THE TARGET POS TO INPUT ##    

        print(f"x: {x} ({type(x)}), y: {y} ({type(y)}), z: {z} ({type(z)})")      
        print(f"current x: {self.NED.x} ({type(self.NED.x)}), current y: {self.NED.y} ({type(self.NED.y)}), current z: {self.NED.z} ({type(self.NED.z)})")           
        print(f"current LAT: {self.GPS.x} ({type(self.GPS.x)}), current LON: {self.GPS.y} ({type(self.GPS.y)}), current ALT: {self.GPS.z} ({type(self.GPS.z)})")           
        if frame:
            self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, self.ze_connection.target_system, 
                self.ze_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                4088, 
                float(x + self.NED.x), 
                float(y + self.NED.y), 
                float((abs(z) * -1)),
               0, 0, 0, 0, 0, 0, 0, 0))   
        else:
            self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, self.ze_connection.target_system, 
                self.ze_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                4088, 
                float(x + self.GPS.x), 
                float(y + self.GPS.y), 
                float(z + self.GPS.z),
                0, 0, 0, 0, 0, 0, 0, 0))   
            
        print(self.wait_4_msg(str_type='COMMAND_ACK', block = True))                       

    async def settle_down(self, tol=0.05):
        ## SETT DRONE BACK TO LAND ##

        # tolerance default is 50mm
        self.set_FComp()
        self.mode = "RTL"
        target_x, target_y, target_z = 0, 0, 0
        self.vel_or_waypoint_mv(x=target_x, y=target_y, z=target_z)
        while not (abs(self.NED.x - target_x) < tol and abs(self.NED.y - target_y) < tol and abs(self.NED.z - target_z) < tol):
            #tolerance same for all         
            if self.mode != 'LAND' and self.battery < 10:
                self.mode = 'LAND'
                target_x, target_y = self.NED.x, self.NED.y
        self.mode = 'STABILIZE'
        await a.to_thread(self.set_wrist(0))                

        
    def to_infinity_and_beyond(self, h=0.25, yaw = 0):   
        ## TAKE OFF AND REACH AN ALTITUDE FOR GUIDED MODE/WHEN STARTING FOR  ##  

        self.mode = 'GUIDED'
        self.set_wrist(1)
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, yaw, 0, 0, h)    
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))          

    def move_servo(self, inst=8, pwm=1500):
        ## MOVE THE MOTOR AT INST A VAL OF PWM ##
        self.ze_connection.mav.command_long_send(
            target_system=self.ze_connection.target_system,
            target_component=self.ze_connection.target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            confirmation=0,
            param1=inst,
            param2=pwm,
            param3=0, param4=0, param5=0, param6=0, param7=0
        )  

    def set_FComp(self):
        ## ALLOW FCOMP TO CONTROL THE DRONE AND NOT GCS ##

        self.ze_connection.mav.command_long_send(
                    self.ze_connection.target_system,
                    self.ze_connection.target_component,
                    mavutil.mavlink.MAV_CMD_DO_PAUSE_CONTINUE,
                    0,
                    0, 0, 0, 0, 0, 0, 0  # param1=0 pauses mission
                ) 
        print(self.wait_4_msg(str_type='COMMAND_ACK', block=True))
        self.target = True
        ## MAV_CMD_NAV_GUIDED_ENABLE 
        self.ze_connection.mav.command_long_send(
                    self.ze_connection.target_system, 
                    self.ze_connection.target_component, 
                    mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE, 
                    0, 1, 0, 0, 0, 0, 0, 0)    
        print(self.wait_4_msg(str_type='COMMAND_ACK', block=True))
        self.mode = 'GUIDED'

        
