import learning as ln
from drone_class import Drone
from time import time

if __name__ == "__main__":
    drone = Drone()
    start = time.time()
    while True:
        drone.move_servo(6, 1500)
        time.sleep(5)
        drone.move_servo(6, 1100)
        time.sleep(5)
        if time.time() - start == 60:
            break