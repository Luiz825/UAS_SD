from pymavlink import mavutil
import arm as a
import learning as ln
import listen as ls  

# takeoff only in the z plane no takeoff to a lat-lon position just goes up
 
def __wait_drama(target): #target must be in terms of meters!!!    
    while True:    
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        pos = msg.z # because in terms of mm initially based on docs        
        if pos == target: break

def to_infinity_and_beyond(h, yaw = 0):     
    a.mode_activate("GUIDED")
    a.set_wrist(1)
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, yaw, 0, 0, h)    
    print(ls.wait_4_ack())
    __wait_drama(h)



