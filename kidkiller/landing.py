import learning as ln
import listen as ls
import arm as a

def settle_down():
    # Get mode ID for GUIDED
    a.mode_activate("LAND")    
    while True:
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")    
        print(f"Current altitude: {msg.z:.2f}")
        if abs(msg.z - 0) < 0.1 : break
    a.set_wrist(0)    
    print(ls.wait_4_ack())
