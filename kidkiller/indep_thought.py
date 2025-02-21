from pymavlink import mavutil
import learning as ln
import listen
import arm
import takeoff
import movement
import back_l2_aunch
import landing

listen.ba_dum()

arm.set_wrist()

takeoff.to_infinity()

movement.king_julian()

back_l2_aunch.ret_to_base()

landing.settle_down()