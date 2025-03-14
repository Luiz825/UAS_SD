import listen as ls
import learning as ln
import os
import time
import csv
from datetime import datetime
import asyncio

async def log_test(filename = "FILE_log", time_out_sec = 15, loop_time_min = 10):
    if not os.path.isfile(filename):
        with open(filename, mode = "w", newLine = ", ") as file:
            scribe = csv.writer(file)
            scribe.writerow("Timestamp", "Timelapse", "Distance")
    
    # will only allow a time of ten minutes of recording data
    
    with open(filename, mode = "a", newLine = ", ") as file:
        scribe = csv.writer(file)
        start_ = time.time()
        while ((time.time() - start_) / 60) < loop_time_min:
            tm, msg = ls.wait_4_msg("LOCAL_POSITION_NED", time_out_sec, 3)
            if tm == time_out_sec:
                break
            timestamp = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            pos = f"x: {msg.x}, y: {msg.y}, z: {msg.z}"
            scribe.writerow([timestamp, tm, pos])

            file.flush()
            asyncio.sleep(3)
    


