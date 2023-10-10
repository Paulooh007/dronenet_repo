import numpy as np
import cv2
import random

import math
from pymavlink import mavutil
import sys
import datetime

import time


class mission_item:
    def __init__(self, l, current, x,y,z, cmd=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, p2=2.00):
        self.seq = l
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = cmd
        self.current = current
        self.auto = 1
        self.param1 = 0.0   #Hold time in decimal seconds
        self.param2 = p2   #Acceptance radius in meters
        self.param3 = 0  #Pass through radius in meters
        self.param4 = 0  #math.nan  #Desired yaw angle in degrees
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print("*-- Sending Message Out*")

    the_connection.mav.mission_count_send(
            the_connection.target_system, 
            the_connection.target_component,
            n,0)
    
    ack("MISSION_REQUEST")

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
        ack("MISSION_REQUEST")

    ack("MISSION_ACK")

def start_mission(the_connection):
    print("*-- Mission Start*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START,
            0,0,0,0,0,0,0,0)
    
    ack("COMMAND_ACK")

def arm():
    print("*-- Arming*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
            )
    
    ack("COMMAND_ACK")

def takeoff(height):
    print("*-- Takeoff Initiated*")

    the_connection.mav.command_long_send(
            the_connection.target_system, 
            the_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,math.nan,0,0,height)
    
    ack("COMMAND_ACK")

def set_flight_mode(fmode):
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

def await_heartbeat():
    while the_connection.target_system == 0:
        print("-- Checking for heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % \
            (the_connection.target_system,
             the_connection.target_component))

def ack(keyword):
    print("-- Message Read " + \
          str(the_connection.recv_match(type=keyword, blocking=True))
          )

def send_body_offset_ned_command(x, y,z, velocity = True):
    # print(f"Sending Velocity commands {x},{y}!")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        10, the_connection.target_system,
                        the_connection.target_component, 
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                        # int(0b110111111000), #pos
                        int(0b110111000111) if velocity else int(0b110111111000), #vel
                        x, y, -1 * z, #position
                        x, y, 0,  #velocity
                        0, 0, 0, 
                        1.56, 0))
    
def distance(pointA, pointB):
    """Get Euclidean distance between 2 points"""
    return (
        ((pointA[0] - pointB[0]) ** 2) +
        ((pointA[1] - pointB[1]) ** 2)
    ) ** 0.5  

def drawMarker(corners, ids):
    """Draw marker bounding box and return the center coordinates, height and width"""
    for (markerCorner, markerID) in zip(corners, ids):
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        markerWidth = int(distance(topLeft,topRight))
        markerHeight = int(distance(topLeft, bottomLeft))

        # draw the bounding box of the ArUCo detection
        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

        # compute and draw the center (x, y)-coordinates of the
        # ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

        # draw the ArUco marker ID on the frame
        cv2.putText(frame, str(markerID),
        (topLeft[0], topLeft[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5, (0, 255, 0), 2)

    return [(cX, cY), (markerWidth, markerHeight)]  #[ midpoint, w and h]

def drawQuadrant():
    """Divide frame into 4 regions/quadrants"""
    cv2.line(frame, (centerX, 0), (centerX,h), (0, 0, 255), 1)
    cv2.line(frame, (0, centerY), (w,centerY), (0, 0, 255), 1)
    cv2.circle(frame, (centerX, centerY), 4, (0, 255, 255), -1)



takeoff_height = 15.0
detection_height = 5.0
x_ang, y_ang = 0, 0

sitl = not False

# parameters gotten from calliberations
intrinsic_camera = np.array(((414.39569534143277, 0.0, 317.9857504092252),\
                             (0,415.1888194356079, 238.29845412845705),(0,0,1)))
distortion = np.array((-0.18568789936480537,
                            0.3161305968015312,
                            0.0018131913124589036,
                            0.004707013829725814,
                            -0.3005001522879405))

# set video resolution. **Must be same value used for caliberation
w, h = 640, 480

# camera field of view.
horizontal_fov = 90 * (math.pi / 180 )
vertical_fov = 90 * (math.pi / 180)

centerX, centerY = w//2, h//2

timestamp = datetime.datetime.now().strftime("%Y%m%d%H%M%S")
debug_image_writer = cv2.VideoWriter(f"vid_record_" + str(timestamp) + ".avi", \
                                     cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),\
                                    25.0,(w,h))

if (__name__ == "__main__"):
    # SITL Simulation or not
    if sitl:
        the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550')
        cap = cv2.VideoCapture(0)
    else:
        the_connection = mavutil.mavlink_connection('COM7', 57600)
        cap = cv2.VideoCapture(1)

    
    await_heartbeat()

    mission_waypoints = []
    mission_waypoints.append(mission_item(0, 0, 7.5197183, 4.5203103, takeoff_height))   #Current Home
    mission_waypoints.append(mission_item(1, 0, 7.5201237, 4.5199567, takeoff_height))   #marker coordinate
    mission_waypoints.append(mission_item(2, 0, 7.5201237, 4.5199567, detection_height)) #marker coordinate at detection height


    upload_mission(the_connection, mission_waypoints)
    
    arm()

    the_connection.motors_armed_wait()

    set_flight_mode('GUIDED')

    counter = 0
    n_frame = 30


    # Take off procedure
    while True:
        msg = the_connection.recv_match(type ='HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)

            if "GUIDED" in mode:
                print("Taking off")
                takeoff(takeoff_height)
                break

    #start mission
    while True:
        msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
        if msg:
            if msg.relative_alt >= ((takeoff_height * 1000) - 20):
                start_mission(the_connection)
                break

    #monitor mission
    while True:
        msg = the_connection.recv_match(type ='MISSION_CURRENT', blocking = False)
        if msg:
            if msg.mission_state and msg.mission_state == 5:  #state==5 mission complete
                set_flight_mode('GUIDED')
                break


    #Start landing algorithm
    while True:
        ret, frame = cap.read()

        frame = cv2.resize(frame, (w,h))

        msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
        if msg:
            cv2.putText(frame, f"Alt: {msg.relative_alt}",
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 1)  
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        drawQuadrant()
        
        #Find aruCo marker in frame
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # print(corners)
        if len(corners) > 0:   #if All 4 corners of marker are detected

            ids = ids.flatten()
            markerInfo = drawMarker(corners, ids)  

            cX, cY = markerInfo[0]   #Get marker midpoint

            # estimates the 3D pose (position and orientation) of a marker in a 3D space based on the given input.
            # it's extracting two specific pieces of data: rvec (rotation vector) and tvec (translation vector).
            # These vectors represent the rotation and translation values respectively, 
            # describing how the marker is oriented and positioned in 3D space.
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02,intrinsic_camera,distortion)

            # draw x,y,z axis on marker
            cv2.aruco.drawDetectedMarkers(frame, corners)

            try:
                cv2.drawFrameAxes(frame,intrinsic_camera,
                                distortion, rvec, tvec, 0.01)
            except Exception as e:
                continue

            x,y,z = tvec[0][0][0],tvec[0][0][1], tvec[0][0][2]
            
            y_sum,x_sum = 0, 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]

            # This gives the center of the marker
            x_avg = x_sum*.25
            y_avg = y_sum*.25

            # These lines calculate the horizontal and vertical angles of the marker in the camera's field of view.
            # x_ang and y_ang represent the angles at which the center of the marker is viewed by the camera within its field of view. 
            # This is used to align the center of the camera with the center of the marker.
            x_ang = (x_avg - w *.5) * (horizontal_fov/w)
            y_ang = (y_avg - h *.5) * (vertical_fov/h)

            x_ang = round(x_ang, 4) 
            y_ang = round(y_ang, 4)
             
            # print(x_ang, y_ang)
            
            error = int(distance((cX, cY), (centerX, centerY)))
            markerWidth, markerHeight = markerInfo[1]
            
            
            cv2.putText(frame, f"X_vel: {x_ang}  Y_vel: {y_ang}  Error: {error}",
                    (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 120), 2)

            cv2.line(frame, (cX, cY), (centerX,centerY), (254, 255, 0), 3)

            if counter % n_frame == 0:
                if error > 50:
                    send_body_offset_ned_command(x_ang, y_ang, 0, velocity= False)

        else:
            x_ang, y_ang = 0, 0
            send_body_offset_ned_command(x_ang, y_ang, 0, velocity= False)

            cv2.putText(frame, f"X_vel: {x_ang}  Y_vel: {y_ang}  Error: No marker found!",
                    (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 244), 2)


        counter += 1
        cv2.imshow('video',frame)
        debug_image_writer.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break




    the_connection.motors_disarmed_wait()
    set_flight_mode("STABILIZE")
