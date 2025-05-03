import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
import time
import sys
import csv
from collections import deque #MAKE INTO ASYNC QUEUE
import asyncio as a
import math
from pymavlink import mavutil


from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp

# Initialize tracking variables
prev_time = time.time()
fps_history = deque(maxlen=10)  # Store last 10 FPS values for smoothing
inference_time_history = deque(maxlen=10)  # Store last 10 inference times
confidence_score_history = deque(maxlen=10)  # Store last 10 confidence scores

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()  

class Camera():
    def __init__(self, drone, d_or_s_or_n=0):
        self.drone = drone
        if d_or_s_or_n not in (0, 1, 2):
            raise ValueError("mode must be an integer: 0, 1, or 2")
        self.dsn = d_or_s_or_n    
        self.drop = False  
        self.gps_points = [(0.0, 0.0) for _ in range(10)]
        self.rec = 0
        self.target=False
        self.app = None
        self.prev_time = time.time()
        self.fps_history = deque(maxlen=10)  # Store last 10 FPS values for smoothing
        self.inference_time_history = deque(maxlen=10)  # Store last 10 inference times
        self.confidence_score_history = deque(maxlen=10)  # Store last 10 confidence scores     

    async def cam_start(self):
        ## START CAMERA FUNCTIONALITY ##

        await a.sleep(5)
        if not os.path.exists("/dev/video0"):
            print("Camera not detected at /dev/video0")
            return      
        print(f"Activate Drone Camera")
        if self.active:
            await a.sleep(0.1)
            user_data = user_app_callback_class()
            try:                
                self.app = GStreamerDetectionApp(self.app_callback, user_data) 
                await a.to_thread(self.app.run())       
                return
            except SystemExit as e:
                print(f"GStreamDetectionApp initialization failed {e}")
                return         

    def pixel_to_meters(self, pixel_x, pixel_y, cam_width_px=4608, cam_height_px=2592 , hfov_deg=66, vfov_deg=41):
        ## CONVERT PIXEL OFFSET OF TARGET TO CENTER OF CAMERA ###

        altitude_m=float(self.drone.GPS.z)
        print(f'altitude_m: {altitude_m} ({type(altitude_m)})', flush=True)

        # Calculate real-world width and height of view
        half_hfov_rad = math.radians(hfov_deg / 2)
        half_vfov_rad = math.radians(vfov_deg / 2)
        print(f"{half_hfov_rad} + {half_vfov_rad}")    

        view_width_m = 2 * altitude_m * math.tan(half_hfov_rad)
        view_height_m = 2 * altitude_m * math.tan(half_vfov_rad)
        print(f"{view_height_m} {view_width_m}")

        # Meters per pixel
        meters_per_pixel_x = view_width_m / cam_width_px
        meters_per_pixel_y = view_height_m / cam_height_px
        print(f"{meters_per_pixel_x} + {meters_per_pixel_y}")

        # Convert pixel offsets to meters
        offset_x_m = pixel_x * meters_per_pixel_x
        offset_y_m = pixel_y * meters_per_pixel_y
        print(f"{offset_x_m} + {offset_y_m}")
        print(f"Target is {offset_x_m} by {offset_y_m} away from camera center")

        ## Returns: (float, float): (x_meters, y_meters) movement in meters ##
        return offset_x_m, offset_y_m
    
    def meters_offset_to_gps(self, offset_north, offset_east):
        ## CONVERT METERS INTO GPS COOR ##

        time.sleep(5)
        R = 6378137.0  # Earth radius in meters
        dLat = offset_north / R
        dLon = offset_east / (R * math.cos(math.pi * self.drone.GPS.x / 180))
        new_lat = self.drone.GPS.x + (dLat * 180 / math.pi)
        new_lon = self.drone.GPS.x + (dLon * 180 / math.pi)

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

        print(f"FPS: {avg_fps} | Inference Time: {avg_inference_time} ms | Avg Confidence: {avg_confidence}")

        # Log FPS, inference time, and confidence score to CSV files
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        print(timestamp)
        # Parse detections
        detection_count = 0
        frame_center_x = width / 2
        frame_center_y = height / 2

        for detection in detections:
            self.drone.set_FComp()
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

            offset_x, offset_y = self.pixel_to_meters(pixel_x=target_x, pixel_y=target_y)
            center_x, center_y = self.pixel_to_meters(pixel_x=frame_center_x, pixel_y=frame_center_y)

            print(f"Center of detection: {target_x, target_y}")
            print(f"Center of frame: {frame_center_x, frame_center_y}")            

            if self.dsn is 2:                
                centered_x = False
                centered_y = False
                
                print(f"offset_x: {offset_x} ({type(offset_x)}), offset_y: {offset_y} ({type(offset_y)})")

                # Movement decisions
                if offset_x <= (center_x + 0.003) and offset_x >= (center_x - 0.003):
                    centered_x = True
                if offset_y <= (center_y + 0.003) and offset_y >= (center_y - 0.003):
                    centered_y = True        

                bullseye =  x_min * width >= threshold_x_min and x_max * width <= threshold_x_max and y_min * height >= threshold_y_min and y_max * height <= threshold_y_max            
                
                if centered_y and centered_x and abs(self.NED.z) <= 0.5 and bullseye:
                    print(f"Dropping payload!")
                    self.payload_sequence()
                    time.sleep(0.5)
                    self.vel_or_waypoint_mv(z=5)            
                    time.sleep(0.5)
                    ## NEED TO SET EHIGHT HIGHER BEFORE GOING BACK ##
                    while abs(self.VEL.x) > 0.5 or abs(self.VEL.y) > 0.5 or abs(self.VEL.z) > 0.5:
                        time.sleep(0.1)
                        continue
                    self.mode = 'RTL'                
                else:
                    print(f"Need to move to the payload!")
                    self.vel_or_waypoint_mv(x=offset_x, y=offset_y, z=0.5)            
                    time.sleep(0.5)
                    while abs(self.VEL.x) > 0.5 or abs(self.VEL.y) > 0.5 or abs(self.VEL.z) > 0.5:
                        time.sleep(0.1)
                        continue
            elif self.dsn is 1:
                print(f"Detected a 24 inch spot")
                (lat, lon) = self.meters_offset_to_gps(offset_y, offset_x)
                print(f"Found here! {(lat, lon)}")
                if self.gps_points[self.rec -1] != (lat, lon):
                    self.gps_points[self.rec] = (lat, lon)
                    self.rec = self.rec + 1
                if self.rec >= 10:
                    self.active = False

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
    

