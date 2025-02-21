from pymavlink import mavutil
import learning as ln
# Start a connection listening on a UDP port
#the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
def ba_dum():    
    ln.the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (ln.the_connection.target_system, ln.the_connection.target_component))
    

def wait_4_msg():
    msg = ln.the_connection.recv_match(type = "HEARTBEAT", blocking = True)
    return msg

def wait_4_ack():
    msg = ln.the_connection.recv_match(type = 'COMMAND_ACK', blocking = True);
    print(msg)
