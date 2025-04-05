import drone_move.learning as ln
import listen as ls
import arm as a
import takeoff as to
import movement as m
import back_l2_aunch as back
import landing as la
import record_time_log_hb as r 
import drone_class as dc
import asyncio
import sys

async def main():
    loop = asyncio.get_event_loop()
    drone = dc.Drone(10)

    await asyncio.gather(
        drone.check_telem(),
        drone.grab_mission_stat(),
        drone.mission_exec(),
        drone.land_question()
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
    with open("/media/cece/DuelData/academic/SDSU/SP2025/COMPE492/STORK_TEST.txt", "w") as f:
        # Redirect stdout to the file
        original_stdout = sys.stdout  # Save original stdout
        sys.stdout = f

        print("This goes to the file")
               
        print(ls.wait_4_msg("HEARTBEAT", block=True))

        asyncio.run(main())

        sys.stdout = original_stdout  # Restore stdout
        
        f.close() 
    
    print(f"done or did nt work lol")

    #need hy



