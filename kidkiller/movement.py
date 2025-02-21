from pymavlink import mavutil
import learning as ln

def king_julian():
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, -3, -4, -5, 0, 0, 0, -45, 0))
