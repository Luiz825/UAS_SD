import asyncio as a
from pymavlink import mavutil
from datetime import datetime
from typing import Literal
import pigpio
import time

class Vehicle:
    VALID_MESSAGES = Literal["HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT",
                          "SYS_STATUS", "MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_CURRENT"]
    VALID_MODES = Literal["GUIDED", "LAND", "RTL", "AUTO", "MANUAL"]

    def __init__(self, conn='udp:localhost:14551', t=5):
        self.x=0
        self.y=0
        self.z=0    
        self.lat=0
        self.lon=0
        self.alt=0    
        self.battery=100
        self.ze_connection=mavutil.mavlink_connection(conn, baud = 57600)
        self.pi = pigpio.pi()
        self.conn_qual = 0 # the lower the better meaning no packets lost 
        self.prev_qual = 0   
        self.mode      
    
    async def check_telem(self):    
        ## CHECK THE TELEMETRY DATA ##
        while self.active:  
            if self.mode == "MANUAL":
                continue              
            t_stat, msg_stat = await a.to_thread(self.wait_4_msg, str_type="SYS_STATUS", 
                                                 time_out_sess = self.t_sess, attempts = 2)
            if msg_stat and t_stat <= self.t_sess:
                if msg_stat.battery_remaining == -1:                     
                    print(f"Battery info unavailable :< ")
                    self.battery=None
                else:                        
                    self.battery = msg_stat.battery_remaining                      
                self.prev_qual = self.conn_qual                                         
                self.conn_qual = msg_stat.drop_rate_comm / 10 #in %   
            else:
                if msg_stat == None:
                    self.conn_qual = 100                
            await a.sleep(1)     

    def wait_4_msg(self, str_type: VALID_MESSAGES, block = False, time_out_sess = None, attempts = 4):    
    ## WAIT FOR A MESSAGE FOR ONE CYCLE OR JUST UNTIL  ##
    #time_out_sess is for total time but for the rp5 its gonna be in ticks 
    #so every tick is 1 microsecond, will be spliced into attempts specificed by user or default 4    
        if time_out_sess is None:
            msg = self.ze_connection.recv_match(type = str_type, blocking = block)
            return msg
        else:                    
            temp = time_out_sess * 1e6/ attempts
            start_ = self.pi.get_current_tick()
            while (self.pi.get_current_tick()- start_) < time_out_sess:
                msg = self.ze_connection.recv_match(type = str_type, blocking = False, timeout = temp)
                if msg is None:
                    continue
                elif msg:                
                    return (self.pi.get_current_tick() - start_), msg
                time.sleep(0.1)
            print("No message")
            return time_out_sess, None # when it took entire time to retrieve message and no message was retrieved      


    async def update_NED(self):
        ## HOLD UNTIL POS REACHED ##           
        rel = "LOCAL_POSITION_NED"         
        msg = self.wait_4_msg(rel, block=True)
        if msg:
            self.x, self.y, self.z = msg.x, msg.y, msg.z        
        print(f"Current Position: x = {self.x:.2f} m, y = {self.y:.2f} m, z = {self.z:.2f} m")                
        
    async def update_GPS(self):
        ## HOLD UNTIL POS REACHED ##         
        rel = "GLOBAL_POSITION_INT"        
        # Wait for initial position
        msg = self.wait_4_msg(rel, block=True)
        if msg:
            self.lat = msg.lat / 1e7
            self.lon = msg.lon / 1e7
            self.alt = msg.alt / 1000.0 
        print(f"Current Position: x = {self.lan:.2f} m, y = {self.lon:.2f} m, z = {self.alt:.2f} m")        
        
    async def change_mode(self):
        ## CHANGE THE MODE OF THE DRONE ##
        mode = self.mode
        while True:
            msg_hb = await a.to_thread(super().wait_4_msg, str_type="HEARTBEAT")
            hb_mode = None
            if msg_hb:
                hb_mode = msg_hb.custom_mode
            await a.sleep(0.1)
            if mode != self.mode or (hb_mode != None and hb_mode != self.mode):
                self.mode_activate(self.mode)
                mode = self.mode
            await a.sleep(0.1) 

    def mode_activate(self, mode_e: VALID_MODES):
        ## CAHNGE THE MODE BASED ON POSSIBLE INPUTS ##
        # Get mode ID for GUIDED
        mode_id = super().ze_connection.mode_mapping()[mode_e]
        # Send mode change request
        super().ze_connection.set_mode(mode_id)
        print(f"Mode changed to {mode_e}!") 
