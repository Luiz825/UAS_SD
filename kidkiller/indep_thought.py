import learning as ln
import listen as ls
import arm as a
import takeoff as to
import movement as m
import back_l2_aunch as back
import landing as la
import record_time_log_hb as r 
import asyncio

async def main():
    loop = asyncio.get_event_loop()

    await asyncio.gather(r.log_test(time_out_sec=24, loop_time_min=2), loop.run_in_executor(m.dir_or_waypoint_mv(xv = 1, yv = 1, zv = -1, yaw = 0)))

# this is testing grounds for different paths and options
to.to_infinity_and_beyond(h = 10, yaw = 0)
asyncio.run(main())

