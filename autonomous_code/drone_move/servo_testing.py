import learning as ln
from drone_class import Drone
import time

conn = '/dev/ttyUSB0' # keep permenant! not changeable! while using uart mod with GND
# the default'udp:localhost:14551 keep for simulations! need to add to sim inputs when simming with 'output add 127.0.0.1:14551' command

if __name__ == "__main__":
    drone = Drone(conn=conn)
    msg = drone.wait_4_msg(str_type="HEARTBEAT", block=True)
    if msg:
        print(msg)
        start = time.time()
        for i in range (10, 11):
            print(f"Check servo: {i}")
            drone.move_servo(i, 850)
            msg_0 = drone.wait_4_msg(str_type="COMMAND_ACK", time_out_sess=4, attempts=2)
            if msg_0:
                print(msg_0)
            time.sleep(0.5)
            drone.move_servo(i, 1550)
            msg_25 = drone.wait_4_msg(str_type="COMMAND_ACK", time_out_sess=4, attempts=2)
            if msg_25:
                print(msg_25)
            time.sleep(0.5)                        
        print("4 seconds")

        ###temp values: OPEN = 850 CLOSE = 1550