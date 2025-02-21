from pymavlink import mavutil
import learning as ln
import listen as ls
import arm as a

def settle_down():
    # Get mode ID for GUIDED
    a.mode_activate("LAND")

    msg = ls.wait_4_msg("LOCAL_POSITION_NED")
    eagle = False
    while not eagle:
        if msg.relative_alt == 0 : eagle = True

    a.set_wrist(0)
    msg = ls.wait_4_ack()
    print(msg)
