from pymavlink import mavutil
import arm 
import learning as ln

def wait_4_drama():
    msg = ln.the_connection.recv_match(type = "ATTITUDE", blocking = True)        
    return msg   
 
def wait_drama(target):
    turkey = False
    while not turkey:    
        msg = wait_4_drama()

        pos = msg.relative_alt / 1000
        
        if pos == target: turkey = True

def to_infinity(h):     
    arm.guide_mode_activate()
    arm.set_wrist()
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, h)    



