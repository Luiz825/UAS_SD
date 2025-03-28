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

    await asyncio.gather(r.log_test(time_out_sec=24, loop_time_min=2), loop.run_in_executor(None, m.vel_or_waypoint_mv, 5, 10))

# this is testing grounds for different paths and options
#a.mode_activate('GUIDED')
print(ls.wait_4_msg("HEARTBEAT"))
to.to_infinity_and_beyond(h = 130)
# back.ret_to_base(start = 1)
asyncio.run(main())



