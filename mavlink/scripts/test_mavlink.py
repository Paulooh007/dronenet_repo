from pymavlink import mavutil

 
# Start a connection listening on a UDP port
# the_connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)
the_connection = mavutil.mavlink_connection('COM7', 57600)
# 115200

# the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550')
# the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat 
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

while True:
    msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    print(msg)

# from dronekit import connect

# # Connect to the Vehicle (in this case a UDP endpoint)
# vehicle = connect('172.26.64.1:14550', wait_ready=True)
# print(vehicle)