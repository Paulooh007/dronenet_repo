import math
from pymavlink import mavutil
import sys
import csv
import datetime
import matplotlib.pyplot as plt
import os


class mission_item:
    def __init__(self, l, current, x,y,z):
        self.seq = l
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0.0   #Hold time in decimal seconds
        self.param2 = 4.00   #Acceptance radius in meters
        self.param3 = 8  #Pass through radius in meters
        self.param4 = math.nan  #Desired yaw angle in degrees
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

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

def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("*-- Sending Message Out*")

    the_connection.mav.mission_count_send(
            the_connection.target_system, 
            the_connection.target_component,
            n,0)
    
    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print("*-- Creating a waypoint*")

        the_connection.mav.mission_item_send(
            the_connection.target_system, 
            the_connection.target_component,
            waypoint.seq,
            waypoint.frame,
            waypoint.command,
            waypoint.current,
            waypoint.auto,
            waypoint.param1,
            waypoint.param2,
            waypoint.param3,
            waypoint.param4,
            waypoint.param5,
            waypoint.param6,
            waypoint.param7,
            waypoint.mission_type
        )

    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")

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

def plot_altitude_data(filename):
    altitudes = []
    relative_altitudes = []
    times = []

    with open(filename, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            altitudes.append(float(row['Altitude']))
            relative_altitudes.append(float(row['Relative Altitude']))
            time_str = row['Time']
            time = datetime.strptime(time_str, '%Y%m%d%H%M%S')
            times.append(time)

    plt.plot(times, altitudes, label='Altitude')
    plt.plot(times, relative_altitudes, label='Relative Altitude')
    plt.xlabel('Time')
    plt.ylabel('Altitude/Relative Altitude')
    plt.title('Altitude Data')
    plt.legend()

    # Save the plot as a PNG file with the same filename
    base_path = os.path.splitext(filename)[0]
    save_path = f"{base_path}.png"
    plt.savefig(save_path)
    plt.close()    

if __name__ == "__main__":
    print("-- Program Started")

    # the_connection = mavutil.mavlink_connection('COM7', 57600) #telemetry
    the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550') #simulator
    # the_connection = mavutil.mavlink_connection('COM7', 57600)
    takeoff_height = 20
    mission_in_progress = False

    timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
    filename = f"altitude_data_{timestamp}.csv"

    # Wait for the first heartbeat 
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))
    
    mission_waypoints = []

    mission_waypoints.append(mission_item(0, 0, 7.5128954, 4.5204995, takeoff_height))

    mission_waypoints.append(mission_item(1, 0, 7.5128472, 4.5209328, 20))
    mission_waypoints.append(mission_item(2, 0, 7.5125243, 4.5207328, 20))
    mission_waypoints.append(mission_item(3, 0, 7.5128940, 4.5204969, 20))

    upload_mission(the_connection, mission_waypoints)

    arm(the_connection)

    the_connection.motors_armed_wait()

    set_flight_mode(the_connection, 'GUIDED')

    while True:
        msg = the_connection.recv_match(type ='HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)

            if "GUIDED" in mode:
                print("Taking off")
                takeoff(the_connection, takeoff_height)
                break

    while True:
        msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
        if msg:
            with open(filename, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)

                if csvfile.tell() == 0:
                    writer.writerow(['Altitude', 'Relative Altitude', 'Time'])

                altitude = msg.alt  # Replace with actual altitude extraction
                relative_altitude = msg.relative_alt  # Replace with actual relative altitude extraction
                current_time = datetime.datetime.now().strftime("%Y%m%d%H%M%S")

                print(altitude, relative_altitude)
                writer.writerow([altitude, relative_altitude, current_time])

            if msg.relative_alt >= ((takeoff_height * 1000) - 200) and (not mission_in_progress):  #20 centimeters shy of take off height
                start_mission(the_connection)
                mission_in_progress = True

        msg = the_connection.recv_match(type ='MISSION_CURRENT', blocking = False)
        
        if msg:
            print(msg)
            if msg.mission_state and msg.mission_state == 5:  #state==5 mission complete
                set_flight_mode(the_connection, "RTL")
                break

    the_connection.motors_disarmed_wait()

    set_flight_mode(the_connection, "STABILIZE")

    print("Reset to Stabilize")
# 172.26.64.1
# 14550

  








        