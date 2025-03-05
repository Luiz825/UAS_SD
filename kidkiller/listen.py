import time
from typing import Literal
import learning as ln

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
def ba_dum():    
    print(wait_4_msg("HEARTBEAT", time_out_sess=30)    )

def wait_4_msg(str_type: Literal["HEARTBEAT", "COMMAND_ACK", "LOCAL_POSITION_NED", "HOME_POSITION"], time_out_sess = None, attempts = 4):    #time_out_sess is for total time, will be spliced into attempts specificed by user or default 4
    if time_out_sess is None:
        msg = ln.the_connection.recv_match(type = str_type, blocking = True)
        return msg
    else:        
        temp = time_out_sess / attempts
        start_ = time.time()        
        while (time.time() - start_) < time_out_sess:
            msg = ln.the_connection.recv_match(type = str_type, blocking = True, timeout = temp)
            if msg is None:
                continue
            elif msg:
                print(msg)
                return
        print("No message")

def wait_4_ack():    
    return wait_4_msg("COMMAND_ACK")
