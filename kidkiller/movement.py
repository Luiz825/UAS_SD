from pymavlink import mavutil
import learning as ln
import listen as ls

def madagascar(x = None, y = None, z = None, xv = None, yv = None, zv = None, xa = None, ya = None, za = None, yaw = 0):
    if (x or y or z):
        __king_julian(x, y, z, yaw)
        __hold_until(x, y, z)
    elif (xv or yv or zv):
        __zumba(xv, yv, zv, yaw)
        __hold_until_v(xv, yv, zv)
    elif (xa or ya or za):
        __hit_the_gas(xa, ya, za, yaw)
    
def __king_julian(x, y, z, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, x, y, z, 0, 0, 0, 0, 0, 0, yaw, 0))

def __zumba(xv, yv, zv, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, xv, yv, zv, 0, 0, 0, yaw, 0))

def __hit_the_gas(xa, ya, za, yaw):
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, 0, 0, 0, xa, ya, za, yaw, 0))

def __hold_until(x, y, z):        
    while True:
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        if (x == msg.x and y == msg.y and z == msg.z):
            break

def __hold_until_v(xv, yv, zv):        
    while True:
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        if (xv == msg.xv and yv == msg.yv and zv == msg.zv):
            break