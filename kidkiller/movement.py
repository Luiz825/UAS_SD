from pymavlink import mavutil
import learning as ln
import listen as ls

def madagascar(x = None, y = None, z = None, xv = None, yv = None, zv = None, xa = None, ya = None, za = None, yaw = 0):
    if (x or y or z):
        __king_julian(x, y, z, yaw)
        hold_until(x, y, z)
    elif (xv or yv or zv):
        __zumba(xv, yv, zv, yaw)
        hold_until_v(xv, yv, zv)
    elif (xa or ya or za):
        __hit_the_gas(xa, ya, za, yaw)
    
def __king_julian(x, y, z, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))

def __zumba(xv, yv, zv, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, xv, yv, zv, 0, 0, 0, yaw, 0))

def __hit_the_gas(xa, ya, za, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, 0, 0, 0, xa, ya, za, yaw, 0))

def hold_until(t_x, t_y, t_z):        
    while True:        
        msg = ls.wait_4_msg("GLOBAL_POSITION_INT")
        x = msg.lat
        y = msg.lon
        z = msg.alt
        print(f"Current Position: Lat {x/1e7}, Lon {y/1e7}, Alt {z/1000}m")
        print(f"Target Position: Lat {t_x/1e7}, Lon {t_y/1e7}, Alt {t_z/1000}m")
        if(abs(t_x - x) < 0.1 and abs(t_y - y) < 0.1 and abs(t_z - z) < 0.1):
            break

def hold_until_v(xv, yv, zv):        
    while True:
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        vel_x = msg.vx
        vel_y = msg.vy
        vel_z = msg.vz
        print(f"Current Velocities: vx={vel_x:.2f}, vy={vel_y:.2f}, vz={vel_z:.2f}")
        print(f"Target Velocities: vx={xv:.2f}, vy={yv:.2f}, vz={zv:.2f}")
        if ((abs(xv - vel_x) < 0.1) and (abs(yv - vel_y) < 0.1) and (abs(zv - vel_z) < 0.1)):
            break