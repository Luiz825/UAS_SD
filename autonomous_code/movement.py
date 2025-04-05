from pymavlink import mavutil
import arm as a
import learning as ln
import listen as ls

#mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

def vel_or_waypoint_mv(frame = 1, x = None, y = None, z = None, xv = None, yv = None, zv = None, yaw = None, mode = "GUIDED"):
    # in terms of meters can input x y or z or xv yv or zv or yaw any is optional but will not take in another input until 
    #this is complete
    if mode != "GUIDED":
        a.mode_activate("GUIDED")
    # if all of these parameters can be organized in a list format
    # [j = getattr(str("pos.")+str(j) if j is None else j)]
    # i also don't really recall if you need to force cast thos strings

    if (x != None or y != None or z != None):
        waypoint_mv(frame, x, y, z, yaw)        
    elif (xv != None or yv != None or zv != None):
        vel_mv(frame, xv, yv, zv, yaw)        
    # elif (xa != None or ya != None or za != None):
    #     __hit_the_gas(xa, ya, za, yaw)    
    
def waypoint_mv(frame, x, y, z, yaw):
    pos = ls.wait_4_msg("LOCAL_POSITION_NED", block = True)
    x = pos.x if x is None else x
    y = pos.y if y is None else y
    z = 0 if z is None else z    
    yaw = ls.wait_4_msg("ATTITUDE", block = True).yaw if yaw is None else yaw
    
    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 4088, x, y, (z + pos.z), 0, 0, 0, 0, 0, 0, yaw, 0))
    hold_until(frame, x, y, (z + pos.z))

def vel_mv(frame, xv, yv, zv, yaw):
    pos = ls.wait_4_msg("LOCAL_POSITION_NED", block = True)
    xv = pos.vx if xv is None else xv
    yv = pos.vy if yv is None else yv
    zv = pos.vx if zv is None else zv
    yaw = ls.wait_4_msg("ATTITUDE", block = True).yaw if yaw is None else yaw

    ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED if frame == 1 else mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, 3527, 0, 0, 0, xv, yv, zv, 0, 0, 0, yaw, 0))
    hold_until_v(frame, xv, yv, zv)

# def __hit_the_gas(xa, ya, za, yaw):
#     ln.the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 3527, 0, 0, 0, 0, 0, 0, xa, ya, za, yaw, 0))
             
def hold_until(frame=1, t_x = None, t_y = None, t_z = None, tol = 0.5):
    print("Begin to waypoint")   
    rel = "LOCAL_POSITION_NED" if frame != 0 else "GLOBAL_POSITION_INT"
    
    # Wait for initial position
    pos = ls.wait_4_msg(rel, block=True)
    if rel == "LOCAL_POSITION_NED":
        t_x = pos.x if t_x is None else t_x
        t_y = pos.y if t_y is None else t_y
        t_z = pos.z if t_z is None else t_z
    else:
        lat = pos.lat / 1e7
        lon = pos.lon / 1e7 
        alt = pos.alt / 1000.0  # assuming millimeters
        t_x = lat if t_x is None else t_x / 1e7
        t_y = lon if t_y is None else t_y / 1e7
        t_z = alt if t_z is None else t_z /1e7

    while True:
        msg = ls.wait_4_msg(rel, block=True)
        if rel == "LOCAL_POSITION_NED":
            x, y, z = msg.x, msg.y, msg.z
        else:
            x = msg.lat / 1e7
            y = msg.lon / 1e7
            z = msg.alt / 1000.0 
        print(f"Current Position: x = {x:.2f} m, y = {y:.2f} m, z = {z:.2f} m")
        print(f"Target Position: x = {t_x:.2f} m, y = {t_y:.2f} m, z = {t_z:.2f} m")
        if(abs(t_x - x) < tol and abs(t_y - y) < tol and abs(t_z - z) < tol):
            print("Position set")  
            return   

#ignore vel commands for now not really necessary

def hold_until_v(frame=1, xv = None, yv = None, zv = None, tol = 0.1):    
    print("Begin to velocity")
    pos = ls.wait_4_msg("LOCAL_POSITION_NED", block = True)
    xv = pos.vx if xv is None else xv
    yv = pos.vy if yv is None else yv
    zv = pos.vz if zv is None else zv    
    while True:        
        msg = ls.wait_4_msg("LOCAL_POSITION_NED", block = True)
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
            return 1  