import learning as ln 
import listen as ls
import landing as l
import movement as m
import asyncio as a
import time
import queue as q

class Drone:
    def __init__(self):
        self.ze_connection = ln.the_connection
        self.waypoint_queue = q.Queue()
        #self.commands_queue = a.Queue() no longer needed        
        self.mission = False
        self.active = True
        self.battery = None
        #self.state = "IDLE" not efficient
        self.conn_qual = 100
        self.t_sess = 10

        async def check_telem(self, batt_lim):
            while self.active:
                t_telem, msg_telem = await a.to_thread(ls.wait_4_msg, str_type="HEARTBEAT", time_out_sess=self.t_sess, attempts=5)
                if msg_telem and t_telem <= self.t_sess:
                    print(msg_telem)
                else:
                    print(f"Broken signal {time.time()}")
                    self.active = False
                    await a.sleep(0.1)
                    continue
                t_stat, msg_stat = await a.to_thread(ls.wait_4_msg, str_type="SYS_STATUS", time_out_sess = self.t_sess, attempts = 2)
                if msg_stat and t_stat <= self.t_sess:
                    if msg_stat.battery_remaining == -1: print("Battery info unavailable :<")
                    else:                        
                        self.battery = msg_stat.battery_remaining
                        print(f"Battery: {self.battery}% at {time.time()}")                       
                    self.conn_qual = msg_stat.drop_rate_comm / 10 #in %                
                await a.sleep(0.5)           

        async def grab_mission_stat(self):
            while self.active:
                t_mission, msg_mission = await a.to_thread(ls.wait_4_msg, str_type="MISSION_ITEM")
                if msg_mission and t_mission <= self.t_sess:
                    self.mission = True
                    self.waypoint_queue.put((msg_mission.frame, msg_mission.x, msg_mission.y, msg_mission.z))
                await a.sleep(0.1)   

        async def mission_exec(self):
            while self.active:
                if self.mission and not self.waypoint_queue.empty():
                    frame, x, y, z = self.waypoint_queue.get()
                    await a.to_thread(m.vel_or_waypoint_mv, frame=frame, x=x, y=y, z=z)
                    await a.sleep(2)
                if self.waypoint_queue.empty():
                    self.mission = False
                    continue
        
        async def land_question(self):
            while self.active:
                if self.battery < 25 and self.conn_quality < 25:
                    self.active = False
                    await a.sleep(1)
            self.mission = False
            await a.to_thread(l.settle_down)
            await a.sleep(0.1)

                    
