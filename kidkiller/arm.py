from pymavlink import mavutil
import learning as ln

# Start a connection listening on a UDP port
#the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
def guide_mode_activate():
    # Get mode ID for GUIDED
    mode_id = ln.the_connection.mode_mapping()["GUIDED"]

    # Send mode change request
    ln.the_connection.set_mode(mode_id)
    print("Mode changed to GUIDED!")   

def set_wrist():
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
    return msg

