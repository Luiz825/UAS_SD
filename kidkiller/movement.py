from pymavlink import mavutil
import arm as a
import learning as ln
import listen as ls

def madagascar(x = None, y = None, z = None, xv = None, yv = None, zv = None, yaw = None):
    a.mode_activate("GUIDED")
    # if all of these parameters can be organized in a list format
    # [j = getattr(str("pos.")+str(j) if j is None else j)]
    # i also don't really recall if you need to force cast thos strings

    if (x != None or y != None or z != None):
        __king_julian(x, y, z, yaw)
        hold_until(x, y, z)
    elif (xv != None or yv != None or zv != None):
        __zumba(xv, yv, zv, yaw)
        hold_until_v(xv, yv, zv)
    # elif (xa != None or ya != None or za != None):
    #     __hit_the_gas(xa, ya, za, yaw)    
    
def __king_julian(x, y, z, yaw):
    pos = ls.wait_4_msg("LOCAL_POSITION_NED")
    x = pos.x if x is None else x
    y = pos.y if y is None else y
    z = pos.z if z is None else z    
    yaw =  (ls.wait_4_msg("ATTITTUDE")).yaw if yaw is None else yaw
    
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 4088, x, y, (z + pos.z), 0, 0, 0, 0, 0, 0, yaw, 0))

def __zumba(xv, yv, zv, yaw):
    pos = ls.wait_4_msg("LOCAL_POSITION_NED")
    xv = pos.vx if xv is None else xv
    yv = pos.vy if yv is None else yv
    zv = pos.vx if zv is None else zv
    yaw =  (ls.wait_4_msg("ATTITTUDE")).yaw if yaw is None else yaw

    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, xv, yv, zv, 0, 0, 0, yaw, 0))

# def __hit_the_gas(xa, ya, za, yaw):
#     ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, 0, 0, 0, xa, ya, za, yaw, 0))

def hold_until(t_x = None, t_y = None, t_z = None, tol = 1):
    pos = ls.wait_4_msg("LOCAL_POSITION_NED")
    t_x = pos.x if t_x is None else t_x
    t_y = pos.y if t_y is None else t_y
    t_z = pos.z if t_z is None else t_z          
def hold_until(t_x, t_y, t_z, tol = 0.5):
    #madagascar(xv = 0, yv = 0, zv = 0)       
    while True:        
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        x = msg.x
        y = msg.y
        z = msg.z
        print(f"Current Position: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}m")
        print(f"Target Position: x = {t_x:.2f}, y = {t_y:.2f}, z = {t_z:.2f}m")
        if(abs(t_x - x) < tol and abs(t_y - y) < tol and abs(t_z - z) < tol):
            print("Position set")
        print(f"Current Position: x {x:.2f}, y {y:.2f}, z {z:.2f}m")
        print(f"Target Position: x {t_x:.2f}, y {t_y:.2f}, z {t_z:.2f}m")
        if(abs(t_x - x) < tol and abs(t_y - y) < tol and abs(t_z - z) < tol):
            print(ls.wait_4_ack())
            break

def hold_until_v(xv = None, yv = None, zv = None, tol = 0.1):    
    pos = ls.wait_4_msg("LOCAL_POSITION_NED")
    xv = pos.x if xv is None else xv
    yv = pos.y if yv is None else yv
    zv = pos.z if zv is None else zv    
    while True:        
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        vel_x = msg.vx
        vel_y = msg.vy
        vel_z = msg.vz
        print(f"Current Velocities: vx = {vel_x:.2f}, vy = {vel_y:.2f}, vz = {vel_z:.2f}")
        print(f"Target Velocities: vx = {xv:.2f}, vy = {yv:.2f}, vz = {zv:.2f}")
        if abs(msg.z) < (5 * tol):
            print("Almost ate dirt!")
            zv = zv - 1;            
        if ((abs(xv - vel_x) < tol) and (abs(yv - vel_y) < tol) and (abs(zv - vel_z) < tol)):
            print("Velocity set")
            return    
        print(f"Current Velocities: vx={vel_x:.2f}, vy={vel_y:.2f}, vz={vel_z:.2f}")
        print(f"Target Velocities: vx={xv:.2f}, vy={yv:.2f}, vz={zv:.2f}")
        if ((abs(xv - vel_x) < 0.1) and (abs(yv - vel_y) < 0.1) and (abs(zv - vel_z) < 0.1)):
            break
