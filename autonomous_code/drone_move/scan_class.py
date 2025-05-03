import time
import csv
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as nP
import hailo
import sys
from collections import deque #MAKE INTO ASYNC QUEUE
from camera_q_class import user_app_callback_class

import builtins
import functools

print = functools.partial(builtins.print, flush=True)

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp
from drone_class import Drone

class Scan(Drone):
    def __init__(self):
        super(Scan, self).__init__()
    
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
                if not self.target:
                    self.set_FComp()         
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