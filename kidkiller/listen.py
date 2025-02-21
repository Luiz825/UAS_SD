from pymavlink import mavutil
import learning as ln

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
def ba_dum():    
    ln.the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (ln.the_connection.target_system, ln.the_connection.target_component))

def wait_4_msg(str_type):    
    return ln.the_connection.recv_match(type = str_type, blocking = True)

def wait_4_ack():    
    return ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
