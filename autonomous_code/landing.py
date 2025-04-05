import learning as ln
import listen as ls
import arm as a
import movement as m

def settle_down():
    # Get mode ID for GUIDED
    a.mode_activate("RTL") 
    m.hold_until(t_z = 0, tol = 1)    
    a.set_wrist(0)    
    a.mode_activate("STABILIZE")
    print(ls.wait_4_msg(str_type="HEARTBEAT", block=True))
