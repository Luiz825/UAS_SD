import drone_class as dc
import asyncio
import sys
import time
from pymavlink import mavutil 

conn = '/dev/ttyUSB0' # keep permenant! not changeable! while using uart mod with GND
# the default'udp:localhost:14551 keep for simulations! need to add to sim inputs when simming with 'output add 127.0.0.1:14551' command

async def main(drone):
    loop = asyncio.get_event_loop()    

    await asyncio.gather(
        drone.log_test(loop_time_min=3) #30 minutes
    )

# this is testing grounds for different paths and options
#a.mode_activate('GUIDED')
if __name__ == '__main__':
    time.sleep(2)
    with open("/home/pi/STORK_TEST_LOG.txt", "w") as f:
        # Redirect stdout to the file
        original_stdout = sys.stdout  # Save original stdout
        sys.stdout = f

        print("This goes to the file")
        
        drone = dc.Drone(10) 

        print(drone.wait_4_msg(str_type="HEARTBEAT", block=True))

        asyncio.run(main(drone))

        sys.stdout = original_stdout  # Restore stdout
        f.flush()
        f.close() 
    
    print(f"done or did not work lol")





