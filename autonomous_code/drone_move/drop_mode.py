import drone_class as dc
import asyncio
import sys
import time
from pymavlink import mavutil
import threading

conn = '/dev/ttyUSB0' # keep permenant! not changeable! while using uart mod with GND
# the default'udp:localhost:14551 keep for simulations! 
# need to add to sim inputs when simming with 'output add 127.0.0.1:14551' command
def run_main(drone):
    asyncio.run(main(drone))

def main(drone):
    loop = asyncio.get_event_loop()    
    try:        
        loop.run_until_complete(asyncio.gather(            
            drone.check_telem(),
            drone.crash_check(),            
            drone.land_question(),
            asyncio.to_thread(drone.change_mode),
            asyncio.to_thread(drone.update_specs)
        ))    
    finally:
        loop.close()

#this will run the following plans:
'''
    1. execute mission 
        1.a.navigation
    2.check telemetry data

if either fail the will be sent to landing protoccol
will begin by starting takeoff protoccol
Question: should it just go straight to landing OR be another async 
that will go back to path if the issue was resolved
'''
if __name__ == '__main__':
    time.sleep(2)
    with open("/home/pi/STORK_TEST_AUTONOMOUS_HOVER_CAM_PAYLOAD.txt", "w") as f:
        # Redirect stdout to the file
        original_stdout = sys.stdout  # Save original stdout
        sys.stdout = f

        print(f"After two seconds this was written to the file in question {time.time()}")
        drone = dc.Drone(conn, 10, 1)
        print(f"{drone.wait_4_msg('HEARTBEAT', block=True)}")      

        async_thread = threading.Thread(target=run_main, args=drone)
        
        async_thread.start()
        drone.cam_start()

        async_thread.join()

        sys.stdout = original_stdout  # Restore stdout
        print(f"done or did nt work lol")
                
    print(f"done or did nt work lol")