from pymavlink import mavutil
import learning as ln
import listen as ls
from typing import Literal


def mode_activate(mode_e: Literal["GUIDED", "LAND", "STABILIZE", "MANUAL", "LOITER"]):
    # Get mode ID for GUIDED
    mode_id = ln.the_connection.mode_mapping()[mode_e]
    # Send mode change request
    ln.the_connection.set_mode(mode_id)
    print(f"Mode changed to {mode_e}!")   

def set_wrist(arm_disarm):
    ln.the_connection.mav.command_long_send(ln.the_connection.target_system, ln.the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, arm_disarm, 0, 0, 0, 0, 0, 0)
    msg = ls.wait_4_ack()
    return msg

def manual_sett():
    ln.the_connection.mav.manual_control_send( ln.the_connection.target_system, 0, 0, 500,  0,  0)

