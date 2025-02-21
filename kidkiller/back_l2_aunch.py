from pymavlink import mavutil
import learning as ln

def ret_to_base():
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
    print(msg)