import math
from pymavlink import mavutil
import sys


def arm(the_connection):
    print("*-- Arming*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
            )
    
    ack(the_connection, "COMMAND_ACK")

def takeoff(the_connection, height):
    print("*-- Takeoff Initiated*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,math.nan,0,0,height)
    
    ack(the_connection, "COMMAND_ACK")

def set_return(the_connection):
    print("*-- Set Return To Launch*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")

def set_flight_mode(the_connection, fmode):
    mode = fmode

    # Check if mode is available
    if mode not in the_connection.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(the_connection.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)

def ack(the_connection, keyword):
    print("-- Message Read " + \
          str(the_connection.recv_match(type=keyword, blocking=True))
          )
    
if __name__ == "__main__":
    print("-- Program Started")

    the_connection = mavutil.mavlink_connection('COM7', 57600) #telemetry
    # the_connection = mavutil.mavlink_connection('COM3', 115200) #telemetry
    # the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550')
    takeoff_height = 10

    # Wait for the first heartbeat 
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))

    arm(the_connection)

    the_connection.motors_armed_wait()

    # set_flight_mode(the_connection, 'GUIDED')

    # while True:
    #     msg = the_connection.recv_match(type ='HEARTBEAT', blocking = False)
    #     if msg:
    #         mode = mavutil.mode_string_v10(msg)
    #         print(mode)

    #         if "GUIDED" in mode:
    #             print("Taking off")
    #             takeoff(the_connection, takeoff_height)
    #             break

    # while True:
    #     msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
    #     if msg:
    #         print(msg.relative_alt)
    #         if msg.relative_alt >= ((takeoff_height * 1000) - 20):
    #             set_flight_mode(the_connection, "RTL")
    #             break

    # the_connection.motors_disarmed_wait()

    # set_flight_mode(the_connection, "STABILIZE")

    # print("Reset to Stabilize")


  








        