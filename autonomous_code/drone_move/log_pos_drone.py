import learning as ln
import drone_class as dc
import asyncio
import sys

async def main():
    loop = asyncio.get_event_loop()

    await asyncio.gather(r.log_test(time_out_sec=24, loop_time_min=2), loop.run_in_executor(None, m.vel_or_waypoint_mv, 5, 10))

# this is testing grounds for different paths and options
#a.mode_activate('GUIDED')
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





