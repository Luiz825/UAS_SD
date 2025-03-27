from pymavlink import mavutil
import arm as a
import movement as m
import learning as ln
import listen as ls  

# takeoff only in the z plane no takeoff to a lat-lon position just goes up
 
def __wait_drama(target): #target must be in terms of meters!!!    
    while True:    
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        pos = abs(msg.z)   
        print(f"Current Altitude: {pos:.2f}m, Target: {target}m")   
        if abs(target - pos) < 0.1 : break # tolerance of 100 mm ?

def to_infinity_and_beyond(h, yaw = 0):     
    a.mode_activate("GUIDED")
    a.set_wrist(1)
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, yaw, 0, 0, h)    
    print(ls.wait_4_msg(str_type="COMMAND_ACK"))
    m.hold_until(t_z = abs(h) * -1)     



