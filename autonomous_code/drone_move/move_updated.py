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
from collections import deque
#from sg_code.UAS.UAS_SD.kidkiller.movement import madagascar

from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp


##   DELETE ##
# Initialize tracking variables
prev_time = time.time()
fps_history = deque(maxlen=10)  # Store last 10 FPS values for smoothing
inference_time_history = deque(maxlen=10)  # Store last 10 inference times
confidence_score_history = deque(maxlen=10)  # Store last 10 confidence scores

# File paths for logging

## DELETE ##
fps_log_file = "fps_log.csv"
inference_time_log_file = "inference_time_log.csv"

# Create CSV files with headers if they don’t exist
if not os.path.exists(fps_log_file):
    with open(fps_log_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "FPS", "Avg Confidence"])

if not os.path.exists(inference_time_log_file):
    with open(inference_time_log_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Inference Time (ms)", "Avg Confidence"])

# -----------------------------------------------------------------------------------------------
# User-defined class to be used in the callback function
# -----------------------------------------------------------------------------------------------
class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42  # Example variable

    def new_function(self):  # Example function
        return "The meaning of life is: "

# -----------------------------------------------------------------------------------------------
# User-defined callback function
# -----------------------------------------------------------------------------------------------
def app_callback(pad, info, user_data):
    global prev_time, fps_history, inference_time_history, confidence_score_history

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
        inference_time_history.append(inference_time)
        
        # Compute average confidence score for detected objects
        confidence_scores = [detection.get_confidence() for detection in detections]
        avg_confidence = sum(confidence_scores) / len(confidence_scores)
        confidence_score_history.append(avg_confidence)

    # Calculate FPS
    current_time = time.time()
    time_diff = current_time - prev_time
    prev_time = current_time

    if time_diff > 0:
        fps = 1 / time_diff
        fps_history.append(fps)  # Store FPS values for averaging
    else:
        fps = 0

    # Compute moving averages
    avg_fps = sum(fps_history) / len(fps_history) if fps_history else 0
    avg_inference_time = sum(inference_time_history) / len(inference_time_history) if inference_time_history else 0
    avg_confidence = sum(confidence_score_history) / len(confidence_score_history) if confidence_score_history else 0

    avg_fps = round(avg_fps, 2)
    avg_inference_time = round(avg_inference_time, 2)
    avg_confidence = round(avg_confidence, 2)

    print(f"FPS: {avg_fps} | Inference Time: {avg_inference_time} ms | Avg Confidence: {avg_confidence}")

    # Log FPS, inference time, and confidence score to CSV files
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    
    with open(fps_log_file, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, avg_fps, avg_confidence])

    with open(inference_time_log_file, "a", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, avg_inference_time, avg_confidence])

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

        print(f"Center of detection: {target_x, target_y}")
        print(f"Center of frame: {frame_center_x, frame_center_y}")

        # Movement decisions
        if target_x < frame_center_x:
            print("Move right")
            #madagascar(1)
        else:
            print("Move left")
            #madagascar(-1)

        if target_y < frame_center_y:
            print("Move up")
            #madagascar(1)
        else:
            print("Move down")
           # madagascar(-1)

        # Get track ID
        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            track_id = track[0].get_id()

        string_to_print += (
            f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n"
        )
        detection_count += 1

    if user_data.use_frame:
        # Display FPS, inference time, and confidence score on the frame
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"FPS: {avg_fps}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Inference Time: {avg_inference_time} ms", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Avg Confidence: {avg_confidence}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

    print(string_to_print)
    return Gst.PadProbeReturn.OK

if __name__ == "__main__":
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)            
    app.run()


