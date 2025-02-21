from pymavlink import mavutil
import learning as ln
import listen as ls

def ret_to_base():
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = ls.wait_4_ack()
    print(msg)