from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('COM7', 57600)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))


mode = 'STABILIZE'

# Get mode ID
# mode_id = the_connection.mode_mapping()[mode]
# print(mode_id)
# # Set new mode

# the_connection.mav.set_mode_send(
#     the_connection.target_system,
#     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#     mode_id)

# Send the arm command
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 0, 0, 0, 0, 0, 0)

# Wait for the command acknowledgment
while True:
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
        print("Arm command acknowledged!")
        print("Result:", mavutil.mavlink.enums['MAV_RESULT'][msg.result].description)
        break
