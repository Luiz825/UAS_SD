import learning as ln
import listen as ls
import asyncio
import arm as a

async def check_telem_loop(time_s=25):
    while True:
        session, msg = ls.wait_4_msg("HEARTBEAT", time_out_sess=time_s, attempts=5)
        if session == time_s or msg is None: # if no message found or the whole time was used
            return 
        msg = ls.wait_4_msg("SYS_STATUS").battery_remaining
        if msg < 25: # battery less than 25% of life
            return
        #if anything is faile then end loop and go out of async