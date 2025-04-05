import learning as ln 
import listen as ls
import landing as l
import movement as m
import asyncio as a
from pymavlink import mavutil
from datetime import datetime

class Drone:
    def __init__(self, t):
        self.ze_connection = ln.the_connection
        self.waypoint_queue = a.Queue()      
        self.mission = False
        self.active = True
        self.battery = 100
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        #% is the % of packages lost 
        self.mode = "STABILIZE"
        self.prev_qual = 0       
        self.t_sess = t

    async def check_telem(self):
        while self.active:                
            t_stat, msg_stat = await a.to_thread(ls.wait_4_msg, str_type="SYS_STATUS", time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1: 
                    print("Battery info unavailable :<")
                else:                        
                    self.battery = msg_stat.battery_remaining                      
                self.prev_qual = self.conn_qual                                         
                self.conn_qual = msg_stat.drop_rate_comm / 10 #in %                               
            await a.sleep(1)           

    async def grab_mission_stat(self):
        while self.active:
            self.ze_connection.mav.mission_request_list_send(
                target_system=self.ze_connection.target_system,
                target_component=self.ze_connection.target_component
            )
            await a.sleep(0.01)
            msg_mission = await a.to_thread(ls.wait_4_msg, str_type="MISSION_COUNT")
            if msg_mission and not self.mission and msg_mission.count > 1:
                self.mission = True
                cnt = msg_mission.count
                print(f"Recieving #{cnt} mission waypoints!")
                for i in range(cnt):
                    self.ze_connection.mav.mission_request_int_send(
                        target_system=self.ze_connection.target_system,
                        target_component=self.ze_connection.target_component,
                        seq=i
                    )
                    await a.sleep(0.01)
                    msg_item = None
                    print(f"Looking for waypoint {i}!")
                    while msg_item is None:
                        print(f"Search for item {i}")
                        msg_item = await a.to_thread(ls.wait_4_msg, str_type = "MISSION_ITEM_INT")
                        await a.sleep(0.1)
                        if not self.active:
                            print("Died in search! :O")
                            return
                    await self.waypoint_queue.put((msg_item.frame, msg_item.x, msg_item.y, msg_item.z, mode=self.mode))
                    print(f"Waypoint {i} recieved!")                
                print(f"All {cnt} items recieved!")
            else:
                print(f"Nothing new/No new mission :<")
            await a.sleep(0.5)   

    async def mission_exec(self):
        while self.active:
            if self.mission and not self.waypoint_queue.empty():                    
                try:
                    print(f"Going to waypoint!")
                    # Try to get an item from the queue with a timeout (e.g., 2 seconds)
                    frame, x, y, z = await a.wait_for(self.waypoint_queue.get(), timeout=2)

                    # Process the waypoint
                    await a.to_thread(m.vel_or_waypoint_mv, frame=frame, x=x, y=y, z=z)

                    # Mark this task as done
                    self.waypoint_queue.task_done()
                    print("Waypoint reached!")
                    
                except a.TimeoutError:
                    # Timeout occurred, no items in the queue within 2 seconds
                    if self.waypoint_queue.empty():
                        self.mission = False                                    
            await a.sleep(2)
    
    async def land_question(self):
        avg_qual = self.conn_qual
        iter = 0
        while self.active:            
            avg_qual = (avg_qual + self.prev_qual + self.conn_qual) / 3
            now = datetime.now()
            print(f"Battery: {self.battery}% at {now.strftime("%Y-%m-%d %H:%M:%S")}")
            print(f"Connection Quality: {avg_qual}% at {now.strftime("%Y-%m-%d %H:%M:%S")}")
            if (self.conn_qual > 75 and avg_qual > 75):
                iter = iter + 1                    
                if (iter == 10):
                    print(f"Comms issue!")
                    self.active = False 
            elif self.battery < 50:
                self.active = False
                print(f"Battery too low!")
            else:
                iter = 0            
            await a.sleep(1)
        print(f"Issue occured!")
        self.mission = False
        await a.to_thread(l.settle_down)
        await a.sleep(5)

                    
