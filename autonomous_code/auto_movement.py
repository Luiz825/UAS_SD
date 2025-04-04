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

    await asyncio.gather(loop.run_in_executor(None, ), loop.run_in_executor(None, m.vel_or_waypoint_mv, 5, 10))

#this will run the following plans:
'''
    1. execute mission 
        1.a.navigation
    2.check telemetry data
if either fail the will be sent to landing protoccol
will begin by starting takeoff protoccol
Question: should it just go straight to landing OR be another async that will go back to path if the issue was resolved
'''
print(ls.wait_4_msg("HEARTBEAT"))
to.to_infinity_and_beyond(h = 130)
# back.ret_to_base(start = 1)
asyncio.run(main())

#need 



