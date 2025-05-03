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
  
#from sg_code.UAS.UAS_SD.kidkiller.movement import madagascar

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
