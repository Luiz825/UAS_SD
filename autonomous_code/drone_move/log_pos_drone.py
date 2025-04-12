import drone_class as dc
import asyncio
import sys
import time

async def main(drone):
    loop = asyncio.get_event_loop()    

    await asyncio.gather(
        drone.log_test(loop_time_sess=30) #30 minutes
    )

# this is testing grounds for different paths and options
#a.mode_activate('GUIDED')
if __name__ == '__main__':
    time.sleep(2)
    with open("/media/cece/DuelData/academic/SDSU/SP2025/COMPE492/STORK_TEST_LOG_FAILS.txt", "w") as f:
        # Redirect stdout to the file
        original_stdout = sys.stdout  # Save original stdout
        sys.stdout = f

        print("This goes to the file")
        
        drone = dc.Drone(10) 

        print(drone.wait_4_msg("HEARTBEAT", block=True))

        asyncio.run(main(drone))

        sys.stdout = original_stdout  # Restore stdout

        f.close() 
    
    print(f"done or did nt work lol")





