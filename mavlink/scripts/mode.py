import math
from pymavlink import mavutil
import time


def arm(the_connection):
    print("*-- Arming*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
            )
    
    ack(the_connection, "COMMAND_ACK")

def takeoff(the_connection):
    print("*-- Takeoff Initiated*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,math.nan,0,0,10)
    
    ack(the_connection, "COMMAND_ACK")


def set_return(the_connection):
    print("*-- Set Return To Launch*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")


def start_mission(the_connection):
    print("*-- Mission Start*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,0,0,0,0,0,0,0)
    
    ack(the_connection, "COMMAND_ACK")



def ack(the_connection, keyword):
    print("-- Message Read " + \
          str(the_connection.recv_match(type=keyword, blocking=True))
          )
    
if __name__ == "__main__":
    print("-- Program Started")

    the_connection = mavutil.mavlink_connection('COM7', 57600) #telemetry
    # the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550')

    # Wait for the first heartbeat 
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))

    # upload_mission(the_connection, mission_waypoints)

    # arm(the_connection)

    mode = 'STABILIZE'

    # Get mode ID
    mode_id = the_connection.mode_mapping()[mode]
    print(mode_id)
    # Set new mode

    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    time.sleep(3)
    


    # takeoff(the_connection)

    # time.sleep(5)

    # start_mission(the_connection)

    # # time.sleep(15)

    # # for mission_item in mission_waypoints:
    # #     print("-- Message Read " +
    # #             str(the_connection.recv_match(
    # #                 type="MISSION_ITEM_REACHED",
    # #                 condition= f"MISSION_ITEM_REACHED.seq == {mission_item.seq}",
    # #                 blocking=True)))
        
    # set_return(the_connection)

    # the_connection.mav.mission_set_current_send(the_connection.target_system, the_connection.target_component, 3)

# 172.26.64.1
# 14550








        