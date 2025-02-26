from pymavlink import mavutil
import learning as ln
import listen as ls
import arm as a
import takeoff as to
import movement as m
import back_l2_aunch as back
import landing as la

# this is testing grounds for different paths and options

msg = ls.ba_dum()
print(msg)

to.to_infinity_and_beyond(h = 10, yaw = 10)
back.save_as()
m.madagascar(xv = 1, yv = 1, zv = -1, yaw = -20)
back.ret_to_base()
m.madagascar(xv = -2, yv = 3, zv = -2, yaw = -20)
back.ret_to_base()
