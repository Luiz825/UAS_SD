from pymavlink import mavutil
import arm as a
import learning as ln
import listen as ls  
 
def wait_drama(target): #target must be in terms of meters!!!    
    while True:    
        msg = ls.wait_4_msg("ATTITUDE")
        pos = msg.relative_alt / 1000 # because in terms of mm initially based on docs        
        if pos == target: break

def to_infinity_and_beyond(h):     
    a.mode_activate("GUIDED")
    a.set_wrist(1)
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, h)    
    wait_drama(h)



