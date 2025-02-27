from pymavlink import mavutil
import learning as ln
import listen as ls
import movement as m

def save_as():
    msg = ls.wait_4_msg("GLOBAL_POSITION_INT")

    # Extract coordinates
    current_lat = msg.lat
    current_lon = msg.lon
    current_alt = msg.alt

    # msg = ls.wait_4_msg("ATTITUDE")
    # current_pitch = msg.pitch
    # current_roll = msg.roll
    # current_yaw = msg.yaw
    home = ls.wait_4_msg("HOME_POSITION")
    print(f"Previous home: Lat {home.latitude/1E7} Lon {home.longitude/1E7} Alt {home.altitude/1000}")
    # Set this as the new home position
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, current_lat, current_lon, current_alt, 0, 0, 0)
    home = ls.wait_4_msg("HOME_POSITION")
    print(f"Current home: Lat {home.latitude/1E7} Lon {home.longitude/1E7} Alt {home.altitude/1000}")
    print(f"Home position set to current location! Compare!: Lat {current_lat/1E7} Lon {current_lon/1E7} Alt {current_alt/1000}")    

def ret_to_base():
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)                
    home = ls.wait_4_msg("HOME_POSITION")
    t_x = home.latitude
    t_y = home.longitude
    t_z = home.altitude
    m.hold_until(t_x, t_y, t_z)
    # no need for a while loop because will be the last instruction
