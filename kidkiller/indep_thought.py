import learning as ln
import listen as ls
import arm as a
import takeoff as to
import movement as m
import back_l2_aunch as back
import landing as la
import time

# this is testing grounds for different paths and options
ls.ba_dum()
to.to_infinity_and_beyond(h = 10, yaw = 0)
m.dir_or_waypoint_mv(xv = 1, yv = 1, zv = -1, yaw = 0)
m.dir_or_waypoint_mv(xv = 2, yv = 3, zv = -2, yaw = 0)
back.ret_to_base()
la.settle_down()
while True:
    continue
