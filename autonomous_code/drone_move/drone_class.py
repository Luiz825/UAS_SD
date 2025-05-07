import vehicle_class as vc
import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import math
import time
import os
import csv
import cv2
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

    def __init__(self, conn, t=10, demo=0):
        super(Drone, self).__init__(conn=conn, t=t)                                    
        self.drop = False  
        self.demo = demo
        self.dict_gps = {"Bullseye": (0.0, 0.0), "48_target": [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)], "24_target1": (0.0, 0.0), "24_target4": (0.0, 0.0), "24_rtarget5": (0.0, 0.0)}
        self.gps_target = (0.0, 0.0)
        self.rec = 0
        self.FC=False
        self.app = None
        self.prev_time = time.time()
        self.fps_history = deque(maxlen=10)  # Store last 10 FPS values for smoothing
        self.inference_time_history = deque(maxlen=10)  # Store last 10 inference times
        self.confidence_score_history = deque(maxlen=10)  # Store last 10 confidence scores 
        self.loop= a.get_event_loop()                 

        print = functools.partial(builtins.print, flush=True)

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
            10000,  # Interval in microseconds (10,000 µs = 10 Hz)
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
                    self.FC = True
                    self.mode = 'RTL'
                    await a.sleep(0.1)
                    self.active = False
                    await a.sleep(0.1) 
                    start_ = 0
                else:
                    start_ = 0
            elif self.battery is not None and self.battery < 20: 
                #fix the battery monitor on the flight controller
                self.FC = True
                await a.sleep(0.1)
                self.active = False
                await a.sleep(0.1)
                print(f"Battery low!")                    
            await a.sleep(1)
        print(f"No longer active, ending processes")
        self.mission = False
        await a.to_thread(self.settle_down)
        await a.sleep(1)
    
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
    
    async def payload_sequence(self, inst=8):
        ## SPECIFIC SEQUENCE OF VALUES FOR PAYLOAD DROP ##
        print("DROP")
        while self.active:
            if self.mode == 'MANUAL':
                await a.sleep((0.1))    
                continue  
            if self.drop:
                await self.move_servo(inst, 850)
                await a.sleep(0.01)
                await self.move_servo(inst, 1550)
                await a.sleep(0.01)
                self.drop = False
            await a.sleep((0.1))    

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
                    self.FC = True
                    self.active = False
                    self.mode = 'LAND'
                    start_ = 0
                else:
                    start_ = 0
            await a.sleep(0.1)
        while (self.NED.y - tol) > tol:
            await a.sleep(0.1)
            continue
        a.to_thread(self.set_wrist(0))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION,
            0,  # Confirmation
            1,  # Termination ON
            0, 0, 0, 0, 0, 0)

        
    def vel_or_waypoint_mv(self, frame = 1, x = None, y = None, z = None):
    ## MOVE DRONE ##

    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete        
        self.FC = True      
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
        if frame:
            print(f"current x: {self.NED.x} ({type(self.NED.x)}), current y: {self.NED.y} ({type(self.NED.y)}), current z: {self.NED.z} ({type(self.NED.z)})")           
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
            print(f"current LAT: {self.GPS.x} ({type(self.GPS.x)}), current LON: {self.GPS.y} ({type(self.GPS.y)}), current ALT: {self.GPS.z} ({type(self.GPS.z)})")           
            ## SET_POSITION_TARGET_GLOBAL_INT
            self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                0, self.ze_connection.target_system, 
                self.ze_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 
                4088, 
                float(x + self.GPS.x), 
                float(y + self.GPS.y), 
                float(z + self.GPS.z),
                0, 0, 0, 0, 0, 0, 0, 0))   
        time.sleep((0.1))    
        
        t, msg = None, None
        while msg == None:
            print(f"No movement acknowledgment yet :/")
            t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block = False)                       
            time.sleep((0.1))    
        print(msg)    

    def yaw_mv(self, yaw = 0):
        ## CHANGE YAW ONLY AS DESIRED
        self.FC = True
        print(f"Current yaw: {self.yaw}")                
        self.ze_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                0, self.ze_connection.target_system, 
                self.ze_connection.target_component, 
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                0b0001101111111111,  #ignore everything BUT yaw
                0, 0, 0, 0, 0, 0, 0, 0, 0, float(yaw), 0))           
        t, msg = None, None
        while msg == None:
            print(f"No movement acknowledgment yet :/")
            t, msg = self.wait_4_msg(str_type='COMMAND_ACK', block = False)                       
            time.sleep(0.1)
        print(msg)
        
    async def settle_down(self, tol=0.05):
        ## SETT DRONE BACK TO LAND ##
        # tolerance default is 50mm
        self.FC = True
        self.mode = "RTL"
        target_x, target_y, target_z = 0, 0, 0        
        while not (abs(self.NED.x - target_x) < tol and abs(self.NED.y - target_y) < tol and abs(self.NED.z - target_z) < tol):
            #tolerance same for all         
            if self.mode != 'LAND' and self.battery < 10:
                self.mode = 'LAND'
                target_x, target_y = self.NED.x, self.NED.y
            await a.sleep(0.1)
        self.mode = 'STABILIZE'
        await a.to_thread(self.set_wrist(0))                
        
    def to_infinity_and_beyond(self, h=0.25, yaw = 0):   
        ## TAKE OFF AND REACH AN ALTITUDE FOR GUIDED MODE/WHEN STARTING FOR  ##  
        self.FC = True
        self.mode = 'GUIDED'
        self.set_wrist(1)
        self.ze_connection.mav.command_long_send(
            self.ze_connection.target_system, 
            self.ze_connection.target_component, 
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
            0, 0, 0, 0, float(self.yaw), 0, 0, h)    
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))          

    async def move_servo(self, inst=8, pwm=1500):
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
        print(self.wait_4_msg(str_type="COMMAND_ACK", block = True))

    def cam_start(self):
        ## START CAMERA FUNCTIONALITY ##

        time.sleep(5)
        if not os.path.exists("/dev/video0"):
            # if no god path for video to start then dont start
            self.active = False #cant start without video
            print("Camera not detected at /dev/video0")
            return      
        print(f"Activate Drone Camera")
        if self.active:
            time.sleep(0.1)            
            user_data = user_app_callback_class()

            try:                
                self.app = GStreamerDetectionApp(self.app_callback, user_data) 
                self.app.run()
                return
            except SystemExit as e:
                print(f"GStreamDetectionApp initialization failed {e}")
                return         

    def pixel_to_meters(self, pixel_x, pixel_y, cam_width_px=3280, cam_height_px=2464 , hfov_deg=62.2, vfov_deg=48.8):
        ## CONVERT PIXEL OFFSET OF TARGET TO CENTER OF CAMERA ###

        altitude_m=float(-self.NED.z)        

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
    
    def meters_offset_to_gps(self, offset_north, offset_east):
        ## CONVERT METERS INTO GPS COOR ##

        time.sleep((2)) 
        R = 6378137.0  # Earth radius in meters
        dLat = offset_north / R
        dLon = offset_east / (R * math.cos(math.pi * self.GPS.x / 180))
        new_lat = self.GPS.x + (dLat * 180 / math.pi)
        new_lon = self.GPS.x + (dLon * 180 / math.pi)

        ## Returns: (float, float): (latitude, longitude) for drone to go ##
        return new_lat, new_lon    

    def app_callback(self, pad, info, user_data):   
        '''
        BEGIN CAMERA FRAME INPUT ANALYSIS AND EITHER:
            SCAN FOR TARGETS AND ONCE FOUND ATTEMPT TO LOWER SELF TO LOCATION AT HEIGHT OF 0.5m
            SCAN FOR TARGETS AND RECORD THE GPS COORDINATES OF ALL LOCATED TARGETS WITH #s
        '''
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
        
        # Parse detections
        detection_count = 0
        frame_center_x = width / 2
        frame_center_y = height / 2

        for detection in detections:            
            print(f"Found item! analysis time!")
            label = detection.get_label()
            bbox = detection.get_bbox()
            confidence = detection.get_confidence()

            x_min = bbox.xmin()
            x_max = bbox.xmax()
            y_min = bbox.ymin()
            y_max = bbox.ymax()
            w = bbox.width()
            h = bbox.height()

            threshold_width = width * 0.4
            threshold_height = height * 0.4

            threshold_x_min = (width - threshold_width) / 2
            threshold_x_max = (width + threshold_width) / 2
            threshold_y_min = (height - threshold_height) / 2            
            threshold_y_max = (height + threshold_height) / 2

            target_x = ((x_min + x_max) / 2) * 1000
            target_y = ((y_min + y_max) / 2) * 1000
            
            #center_x, center_y = self.pixel_to_meters(pixel_x=frame_center_x, pixel_y=frame_center_y)

            string_to_print += (f"Center of detection: {target_x, target_y}\n")
            string_to_print += (f"Center of frame: {frame_center_x, frame_center_y}\n")    
            string_to_print += (f"Height: {self.NED.z} ({type(self.NED.z)})")

            offset_x, offset_y = self.pixel_to_meters(pixel_x=target_x, pixel_y=target_y)
            string_to_print += (f"offset_x: {offset_x} ({type(offset_x)}), offset_y: {offset_y} ({type(offset_y)})\n")                  
            self.gps_target = self.meters_offset_to_gps(offset_y, offset_x)
            string_to_print += (f"gps longitude: {self.gps_target[0]} ({type(self.gps_target[0])}), gps latitude: {self.gps_target[0]} ({type(self.gps_target[1])})\n")    

            # Get track ID
            track_id = 0
            track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
            if len(track) == 1:
                track_id = track[0].get_id()

            if label == 'Bullseye':                
                centered_x = False
                centered_y = False
                                
                bullseye =  (x_min * width >= threshold_x_min and 
                             x_max * width <= threshold_x_max and 
                             y_min * height >= threshold_y_min and 
                             y_max * height <= threshold_y_max)
                
                if (centered_y and centered_x and abs(self.NED.z) <= 600 and bullseye):
                    string_to_print += (f"Dropping payload!\n")
                    self.drop = True
                    time.sleep((0.01)) 
                    self.vel_or_waypoint_mv(z=5)  
                    while abs(self.VEL.z) > 5 and not self.demo:
                        time.sleep((0.01)) 
                        continue        
                    time.sleep(0.01)
                    ## NEED TO SET HEIGHT HIGHER BEFORE GOING BACK ##                                          
                    self.mode = 'RTL'   
                    time.sleep(0.1)                                              
                elif self.dict_gps[label] == (0.0, 0.0):
                    string_to_print += (f"Need to move to the target!\n")                          
                    time.sleep((0.01))                                                                   
                    self.vel_or_waypoint_mv(frame=0, x=self.gps_target[0], y=self.gps_target[1], z=550)            
                    time.sleep((0.01)) 

            else:
                string_to_print += (f"Detected a spot!\n")
                (lat, lon) = self.meters_offset_to_gps(offset_y, offset_x)
                string_to_print += (f"Found here! {(lat, lon)} {label} \n") 
                if label == "48_target" and self.rec < 4:
                    if self.dict_gps[label][self.rec] == (0.0, 0.0):
                        self.dict_gps[label][self.rec] = (lat, lon)
                        self.rec += 1               
                        pixel_offset = target_x - frame_center_x
                        deg_per_pix = 62.2 / w
                        angle_to_target_deg = pixel_offset * deg_per_pix
                        #turn to radians 
                        angle_to_target_rad = math.radians(angle_to_target_deg)
                        #based on current yaw grab this
                        desired_yaw = float(self.yaw + math.pi + angle_to_target_rad) % (2 * math.pi)                        
                        time.sleep((0.01)) 
                        self.yaw_mv(yaw=desired_yaw)
                if self.dict_gps[label] == (0.0, 0.0):                    
                    self.dict_gps[label] = (lat, lon)   
                                 
            string_to_print += (
                f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
            )
            detection_count += 1            
                

        if user_data.use_frame:
            string_to_print += (f"{frame} Detections: {detection_count}\n")
            string_to_print += (f"{frame} FPS: {avg_fps}")
            string_to_print += (f"{frame} Inference Time: {avg_inference_time} ms\n")
            string_to_print += (f"{frame} Avg Confidence: {avg_confidence}\n")
            filename = f"/home/pi/UAS_SD/autonomous_code/drone_move/captured_frames/frame_{frame_count}.jpg"  # Make sure the folder exists
            cv2.imwrite(filename, frame)
            string_to_print += (f"Saved frame to {filename}")
            user_data.set_frame(frame)

        string_to_print += (
                f"FPS: {avg_fps} | Inference Time: {avg_inference_time} ms | Avg Confidence: {avg_confidence}\n"
            )
        # Log FPS, inference time, and confidence score to CSV files  
        time.sleep((0.01))      
        if frame_count % 5 == 0:            
            print(string_to_print)
        
        
        return Gst.PadProbeReturn.OK        
    