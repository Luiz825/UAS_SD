import learning as ln
from drone_class import Drone
import time

if __name__ == "__main__":
    drone = Drone()
    msg = drone.wait_4_msg(str_type="HEARTBEAT", block=True)
    if msg:
        print(msg)
        start = time.time()
        for i in range (8, 11):
            print(f"Check servo: {i}")
            drone.move_servo(i, 1000)
            msg_0 = drone.wait_4_msg(str_type="COMMAND_ACK", time_out_sess=4, attempts=2)
            if msg_0:
                print(msg_0)
            time.sleep(0.5)
            drone.move_servo(i, 1500)
            msg_25 = drone.wait_4_msg(str_type="COMMAND_ACK", time_out_sess=4, attempts=2)
            if msg_25:
                print(msg_25)
            time.sleep(0.5)                        
        print("4 seconds")