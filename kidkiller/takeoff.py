from pymavlink import mavutil
import learning as ln

def to_infinity():
    # Get mode ID for GUIDED
    mode_id = ln.the_connection.mode_mapping()["GUIDED"]

    # Send mode change request
    ln.the_connection.set_mode(mode_id)
    print("Mode changed to GUIDED!")

    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
    print(msg)

    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    msg = ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
    print(msg)


