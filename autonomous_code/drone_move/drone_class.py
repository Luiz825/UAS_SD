import vehicle_class as vc
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import math
import time
import os
import csv
import math
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as nP
import hailo
import sys
from collections import deque #MAKE INTO ASYNC QUEUE
from camera_q_class import user_app_callback_class, app_callback

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
        super(Drone, self).__init__(conn, t)        
        self.roll=0
        self.pitch=0                                
        # % is the % of packages lost                 
        self.drop = False   
        self.app = None
        self.prev_time = time.time()
        self.fps_history = deque(maxlen=10)  # Store last 10 FPS values for smoothing
        self.inference_time_history = deque(maxlen=10)  # Store last 10 inference times
        self.confidence_score_history = deque(maxlen=10)  # Store last 10 confidence scores     
    
    async def land_question(self):
        ## CHECK IF NEED TO LAND ##
        avg_qual = self.conn_qual  
        start_ = 0      
        while self.active:   
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue           
            avg_qual = (avg_qual + self.prev_qual + self.conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {self.battery}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")           
            print(f"Connection Quality: {avg_qual}% at {now.strftime('%Y-%m-%d %H:%M:%S')}")            
            if (self.conn_qual > 40 and avg_qual > 40):
                if start_ == 0:
                    #start_ = self.pi.get_current_tick()
                    start_ = time.time()
                #elif (self.pi.get_current_tick() - start_) > 2500:
                elif (time.time() - start_) > 25:
                    print(f"Comms issue!")
                    self.active = False 
                    start_ = 0
                else:
                    start_ = 0
            elif self.battery is not None and self.battery < 20: 
                #fix the battery monitor on the flight controller
                self.active = False
                print(f"Battery low!")                    
            await a.sleep(1)
        print(f"Issue occured!")
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
        # will only allow a time of ten minutes (default) of recording data
        with open(filename, mode = "a", newline = "") as file:
            scribe = csv.writer(file)
            #start_ = self.pi.get_current_tick()
            start_ = time.time()
            #while (((self.pi.get_current_tick() - start_) * 1e6)/60) < loop_time_min:
            while ((time.time() - start_)/60) < loop_time_min:
                now = datetime.now()             
                if not conn:
                    tm, msg = await a.to_thread(self.wait_4_msg,"LOCAL_POSITION_NED", 
                                                time_out_sess=self.t_sess, attempts=3)
                    if tm == self.t_sess:
                        break
                    timestamp = now.strftime("%Y/%m/%d %H:%M:%S")
                    pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
                    scribe.writerow([timestamp, tm, pos])
                else:
                    tm, msg = await a.to_thread(self.wait_4_msg,"RAW_IMU", 
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
        while self.active:
            if self.mode == "MANUAL":
                await a.sleep(0.01)
                continue  
            if self.drop:
                await a.to_thread(self.move_servo, inst, 850)
                await a.sleep(0.5)
                await a.to_thread(self.move_servo, inst, 1550)
                await a.sleep(0.5)
                self.drop = False
            await a.sleep(0.1)

    async def crash_check(self, tol = 0.5):
        ## IF THE DRONE SHIFTS EXTREME TO ANGLE GRATER 100D THEN STOP TO LAND##
        # crashing land tolerance of 1/2 meter #
        start_ = 0
        while self.active:
            if self.roll > 70 or self.pitch > 70:
                if start_ is 0:
                    #start_ = self.pi.get_current_tick()
                    start_ = time.time()
                #elif (self.pi.get_current_tick() - start_) > 300:
                elif (time.time() - start_) > 2:
                    self.active = False
                    self.mode = "LAND"
                    start_ = 0
                else:
                    start_ = 0
            await a.sleep(0.1)
        while (self.y - tol) > tol:
            continue

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0,  # Confirmation
            1,  # Termination ON
            0, 0, 0, 0, 0, 0)
    
    async def update_GYRO(self):
        while self.active:    
            ## HOLD UNTIL POS REACHED ##  
            await a.sleep(0.1)         
            rel = "ATTITUDE"         
            msg = await a.to_thread(self.wait_4_msg, str_type=rel)
            if msg:
                self.roll = msg.roll * 100 / math.pi
                self.pitch = msg.pitch * 100 / math.pi
            print(f"Current Orientation: roll = {self.roll:.2f} m, pitch = {self.pitch:.2f} m") 
        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None, yaw = None):
    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete
    ## MOVE DRONE ##
        if self.mode != "GUIDED":
            self.mode = "GUIDED"
        # if all of these parameters can be organized in a list format
        # [j = getattr(str("pos.")+str(j) if j is None else j)]
        # i also don't really recall if you need to force cast thos strings
        time.sleep(1)
        if (x != None or y != None or z != None):
            self.waypoint_mv(frame, x, y, z, yaw)
        
    def waypoint_mv(self, frame=1, x=0, y=0, z=0, yaw=0):
        ## CHANGE THE TARGET POS TO INPUT ##               
        if frame:
            self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, self.ze_connectionthe_connection.target_system, 
                self.ze_connection.the_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                4088, 
                (x + self.x), 
                (y + self.y), 
                ((z + abs(self.z))*-1),
                0, 0, 0, 0, 0, 0, yaw, 0))   
        else:
            self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, self.ze_connectionthe_connection.target_system, 
                self.ze_connection.the_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                4088, 
                (x + self.lon), 
                (y + self.lat), 
                (z + self.alt),
                0, 0, 0, 0, 0, 0, yaw, 0))   
            
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))                       

    async def settle_down(self, tol=0.05):
        ## SETT DRONE BACK TO LAND ##
        # tolerance default is 50mm
        self.mode = "RTL"
        target_x, target_y, target_z = 0, 0, 0
        while not abs(self.x - target_x) < tol and abs(self.y - target_y) < tol and abs(self.z - target_z) < tol:
            #tolerance same for all         
            if self.mode != "LAND" and self.battery < 10:
                self.mode = "LAND"
                target_x, target_y = self.x, self.y
        self.mode = "STABILIZE"
        await a.to_thread(self.set_wrist(0))                

        
    def to_infinity_and_beyond(self, h=0.25, yaw = 0):   
        ## TAKE OFF AND REACH AN ALTITUDE FOR GUIDED MODE/WHEN STARTING FOR  ##  
        self.mode = "GUIDED"
        self.set_wrist(1)
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, yaw, 0, 0, h)    
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))          

    def move_servo(self, inst, pwm):
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

    async def cam_start(self):
        user_data = user_app_callback_class()
        self.app = GStreamerDetectionApp(user_data, self.app_callback) 
        await a.to_thread(self.app.run())
        while self.active:
            await a.sleep(0.1)
            continue

    def pixel_to_meters(self, pixel_x, pixel_y, cam_width_px=4608, cam_height_px=2592 , hfov_deg=66, vfov_deg=41):
        ## Convert pixel offset from center into meters on ground ###
        altitude_m=self.NED.z * -1
        # Calculate real-world width and height of view
        half_hfov_rad = math.radians(hfov_deg / 2)
        half_vfov_rad = math.radians(vfov_deg / 2)

        view_width_m = 2 * altitude_m * math.tan(half_hfov_rad)
        view_height_m = 2 * altitude_m * math.tan(half_vfov_rad)

        # Meters per pixel
        meters_per_pixel_x = view_width_m / cam_width_px
        meters_per_pixel_y = view_height_m / cam_height_px

        # Convert pixel offsets to meters
        offset_x_m = pixel_x * meters_per_pixel_x
        offset_y_m = pixel_y * meters_per_pixel_y

        ## Returns: (float, float): (x_meters, y_meters) movement in meters ##
        return offset_x_m, offset_y_m

    def app_callback(self, pad, info, user_data):            
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        user_data.increment()
        frame_count = user_data.get_count()
        string_to_print = f"Frame count: {frame_count}\n"

        # Get the video format details
        format, width, height = get_caps_from_pad(pad)

        frame = None
        if user_data.use_frame and format and width and height:
            frame = get_numpy_from_buffer(buffer, format, width, height)

        # Get detections and start timing inference
        roi = hailo.get_roi_from_buffer(buffer)

        start_inference_time = time.time()
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
        end_inference_time = time.time()

        # Calculate inference time
        inference_time = (end_inference_time - start_inference_time) * 1000  # Convert to ms
        avg_confidence = 0  # Default value in case of no detections

        if len(detections) > 0:
            self.inference_time_history.append(inference_time)
            
            # Compute average confidence score for detected objects
            confidence_scores = [detection.get_confidence() for detection in detections]
            avg_confidence = sum(confidence_scores) / len(confidence_scores)
            self.confidence_score_history.append(avg_confidence)

        # Calculate FPS
        current_time = time.time()
        time_diff = current_time - self.prev_time
        self.prev_time = current_time

        if time_diff > 0:
            fps = 1 / time_diff
            self.fps_history.append(fps)  # Store FPS values for averaging
        else:
            fps = 0

        # Compute moving averages
        avg_fps = sum(self.fps_history) / len(self.fps_history) if self.fps_history else 0
        avg_inference_time = sum(self.inference_time_history) / len(self.inference_time_history) if self.inference_time_history else 0
        avg_confidence = sum(self.confidence_score_history) / len(self.confidence_score_history) if self.confidence_score_history else 0

        avg_fps = round(avg_fps, 2)
        avg_inference_time = round(avg_inference_time, 2)
        avg_confidence = round(avg_confidence, 2)

        print(f"FPS: {avg_fps} | Inference Time: {avg_inference_time} ms | Avg Confidence: {avg_confidence}")

        # Log FPS, inference time, and confidence score to CSV files
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(timestamp)
        # Parse detections
        detection_count = 0
        frame_center_x = width / 2
        frame_center_y = height / 2

        for detection in detections:
            label = detection.get_label()
            bbox = detection.get_bbox()
            confidence = detection.get_confidence()

            x_min = bbox.xmin()
            x_max = bbox.xmax()
            y_min = bbox.ymin()
            y_max = bbox.ymax()
            w = bbox.width()
            h = bbox.height()
            target_x = ((x_min + x_max) / 2) * 1000
            target_y = ((y_min + y_max) / 2) * 1000

            offset_x, offset_y = self.pixel_to_meters(pixel_x=target_x, pixel_y=target_y, cam_height_px=h, cam_width_px=w)
            center_x, center_y = self.pixel_to_meters(pixel_x=center_x, pixel_y=center_y, cam_height_px=h, cam_width_px=w)

            print(f"Center of detection: {target_x, target_y}")
            print(f"Center of frame: {frame_center_x, frame_center_y}")

            centered_x = False
            centered_y = False
            # Movement decisions
            if center_x == (offset_x + 0.3) or center_x == (offset_x - 0.3):
               centered_x = True
            if center_y == (offset_y + 0.3) or center_y == (offset_y - 0.3):
                centered_y = True         
            
            if centered_y and centered_x:
                self.payload_sequence()
                time.sleep(0.5)
                self.mode = "RTL"                
            else:
                self.vel_or_waypoint_mv(x=offset_x, y=offset_y, z=0.5)            
                time.sleep(0.5)
                while abs(self.VEL.x) > 0.5 and abs(self.VEL.y) > 0.5 and abs(self.VEL.z) > 0.5:
                    continue

            # Get track ID
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()

            string_to_print += (
                f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
            )
            detection_count += 1

            print( f"{frame} Detections: {detection_count}")
            print( f"{frame} FPS: {avg_fps}")
            print( f"{frame} Inference Time: {avg_inference_time} ms")
            print( f"{frame} Avg Confidence: {avg_confidence}")

        if user_data.use_frame:
            user_data.set_frame(frame)

        print(string_to_print)
        return Gst.PadProbeReturn.OK        
    