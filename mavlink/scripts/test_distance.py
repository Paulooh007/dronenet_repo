import time
from pymavlink import mavutil

# Set the connection parameters (update the device path and baud rate according to your setup)
connection_string = '/dev/ttyUSB0'
baud_rate = 57600

# Connect to the vehicle
print('Connecting to the vehicle on: %s' % connection_string)
vehicle = mavutil.mavlink_connection('udpin:172.26.64.1:14550')

# Wait for the vehicle to connect
while not vehicle.isconnected():
    time.sleep(1)

# Arm the vehicle
vehicle.arducopter_arm()

# Takeoff to a specified altitude (update the altitude value)
target_altitude = 10  # meters
vehicle.simple_takeoff(target_altitude)

# Wait for the vehicle to reach the target altitude
while vehicle.location.global_relative_frame.alt < target_altitude * 0.95:
    time.sleep(1)

print('Vehicle reached target altitude!')

# # Set the target latitude and longitude coordinates (update the coordinates accordingly)
# target_latitude = 37.123456  # degrees
# target_longitude = -122.654321  # degrees

# # Go to the target coordinates
# vehicle.simple_goto(target_latitude, target_longitude)

# # Wait for the vehicle to reach the target coordinates
# while vehicle.mode.name == 'GUIDED':
#     current_latitude = vehicle.location.global_relative_frame.lat
#     current_longitude = vehicle.location.global_relative_frame.lon
#     distance = mavutil.mavlink_distance(target_latitude, target_longitude, current_latitude, current_longitude)

#     if distance < 1:  # meters
#         print('Vehicle reached the target coordinates!')
#         break

#     time.sleep(1)

# # Return to the initial position (home location)
# vehicle.mode = mavutil.mavlink.MAV_MODE_AUTO_RTL

# # Wait for the vehicle to reach the home location
# while vehicle.mode.name == 'RTL':
#     current_latitude = vehicle.location.global_relative_frame.lat
#     current_longitude = vehicle.location.global_relative_frame.lon
#     distance = mavutil.mavlink_distance(home_latitude, home_longitude, current_latitude, current_longitude)

#     if distance < 1:  # meters
#         print('Vehicle returned to the initial position!')
#         break

#     time.sleep(1)

# # Disarm the vehicle
# vehicle.arducopter_disarm()

# # Close the connection
# vehicle.close()
