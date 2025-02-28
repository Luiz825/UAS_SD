from pymavlink import mavutil
import arm as a
import learning as ln
import listen as ls

def madagascar(x = None, y = None, z = None, xv = None, yv = None, zv = None, xa = None, ya = None, za = None, yaw = None):
    a.mode_activate("GUIDED")
    if yaw is None:
        yaw = (ls.wait_4_msg("ATTITTUDE")).yaw
    if (x or y or z):
        __king_julian(x, y, z, yaw)
        hold_until(x, y, z)
    elif (xv or yv or zv):
        __zumba(xv, yv, zv, yaw)
        hold_until_v(xv, yv, zv)
    elif (xa or ya or za):
        __hit_the_gas(xa, ya, za, yaw)
    a.mode_activate("LOITER")
    
def __king_julian(x, y, z, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))

def __zumba(xv, yv, zv, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, xv, yv, zv, 0, 0, 0, yaw, 0))

def __hit_the_gas(xa, ya, za, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, 0, 0, 0, xa, ya, za, yaw, 0))

def hold_until(t_x, t_y, t_z, tol = 0.5):
    #madagascar(xv = 0, yv = 0, zv = 0)       
    while True:        
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        x = msg.x
        y = msg.y
        z = msg.z
        print(f"Current Position: x {x:.2f}, y {y:.2f}, z {z:.2f}m")
        print(f"Target Position: x {t_x:.2f}, y {t_y:.2f}, z {t_z:.2f}m")
        if(abs(t_x - x) < tol and abs(t_y - y) < tol and abs(t_z - z) < tol):
            print(ls.wait_4_ack())
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
