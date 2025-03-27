import movement as m
import learning as ln
import listen as ls
import math
import time
from PID import PID

def move_point(x, y, z, dur = None):
    kp = 0.1
    ki = 0.1
    kd = 0.1
    pid_x = PID(kp, ki, kd, x)
    pid_y = PID(kp, ki, kd, y)
    pid_z = PID(kp, ki, kd, z)

    while dur == None:
        msg = ls.wait_4_msg("LOCAL_POSITION_NED")
        curr_x = msg.x
        curr_y = msg.y
        curr_z = msg.z

        diff_x = pid_x.compute(curr_x, time.time())
        diff_y = pid_y.compute(curr_y, time.time())
        diff_z = pid_z.compute(curr_z, time.time())

        



