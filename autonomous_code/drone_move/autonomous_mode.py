import listen as ls
import drone_class as dc
import asyncio
import sys
import time

async def main():
    loop = asyncio.get_event_loop()
    drone = dc.Drone(10)

    await asyncio.gather(
        drone.check_telem(),
        drone.grab_mission_stat(),
        drone.mission_exec(),
        drone.land_question(),
        drone.change_mode()
    )
    

#this will run the following plans:
'''
    1. execute mission 
        1.a.navigation
    2.check telemetry data
if either fail the will be sent to landing protoccol
will begin by starting takeoff protoccol
Question: should it just go straight to landing OR be another async that will go back to path if the issue was resolved
'''
if __name__ == '__main__':
    time.sleep(10)
    with open("/media/cece/DuelData/academic/SDSU/SP2025/COMPE492/STORK_TEST.txt", "w") as f:
        # Redirect stdout to the file
        original_stdout = sys.stdout  # Save original stdout
        sys.stdout = f

        print("After ten seconds this was written to the file in question")
               
        print(ls.wait_4_msg("HEARTBEAT", block=True))

        asyncio.run(main())

        sys.stdout = original_stdout  # Restore stdout
        print(f"done or did nt work lol")
                
    print(f"done or did nt work lol")

    #need hy



