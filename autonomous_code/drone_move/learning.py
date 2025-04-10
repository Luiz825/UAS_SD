from pymavlink import mavutil 

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud = 57600) # keep permenant! not changeable! while using uart mod with GND
#the_connection = mavutil.mavlink_connection('udp:localhost:14551') # keep for simulations! need to add to sim inputs when simming with 'output add 127.0.0.1:14551' command

