import time
from pymavlink import mavutil

# Set the desired distance to move in meters
distance = 10

# Connect to the drone
master = mavutil.mavlink_connection('udpin:172.26.64.1:14550')  # Replace with your connection details

# Wait for the heartbeat from the drone
master.wait_heartbeat()

# Arm the drone
master.arducopter_arm()

# Take off
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0)

# Wait for the drone to reach the desired altitude
time.sleep(5)  # Adjust as needed

# Calculate the target position
current_position = master.messages['GLOBAL_POSITION_INT']
target_position = [current_position.lat, current_position.lon, current_position.alt]  # Convert to list

# Move forward
target_position[0] += int(distance * 1e7)  # Convert distance to centimeters

# Send the move command
master.mav.set_position_target_local_ned_send(
    0,  # Time since system boot, not used
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Use position setpoint, ignore the rest
    *tuple(target_position))  # Convert back to tuple

# Wait for the drone to move forward
time.sleep(10)  # Adjust as needed

# Move back to the original position
target_position[0] -= int(distance * 1e7)  # Convert distance to centimeters

# Send the move command
master.mav.set_position_target_local_ned_send(
    0,  # Time since system boot, not used
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Use position setpoint, ignore the rest
    *tuple(target_position))  # Convert back to tuple

# Wait for the drone to move back
time.sleep(10)  # Adjust as needed

# Land the drone
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

# Disarm the drone
master.arducopter_disarm()

# Close the connection
master.close()
