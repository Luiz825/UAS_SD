import learning as ln
import listen as ls
import arm as a
import takeoff as to
import movement as m
import back_l2_aunch as back
import landing as la
import time

# this is testing grounds for different paths and options
print(ls.ba_dum())
print(a.mode_activate("STABILIZE"))

to.to_infinity_and_beyond(h = 10, yaw = 0)

a.mode_activate("LOITER")
a.manual_sett()
time.sleep(30)

m.madagascar(xv = 1, yv = 1, zv = 1, yaw = 0)
back.save_as()
m.madagascar(xv = 2, yv = 3, zv = 2, yaw = 0)
