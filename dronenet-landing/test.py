from dronekit import connect, VehicleMode
from pymavlink import mavutil

# def connectMyCopter():
# 	connection_string= "127.0.0.1:5501"
# 	connection_string= "udp:172.26.64.1:14550"
# 	connection_string= "172.26.64.1:14550"
# 	# connection_string= "COM7"
# 	vehicle = connect(connection_string,wait_ready=True, baud=57600, timeout=30)

# 	return vehicle

connection_string= "172.26.64.1:14550"	
connection_string= "tcp:127.0.0.1:5760"	
vehicle = connect(connection_string,wait_ready=True, baud=57600, timeout=30)

# vehicle = connectMyCopter()
print(vehicle)
print(f"Armable: {vehicle.is_armable}")

# arm_and_takeoff(10)