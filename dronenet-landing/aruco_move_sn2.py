import numpy as np
import cv2
import random
import time

import math
from pymavlink import mavutil
import sys


w, h = 640, 480
takeoff_height = 15
velocity = 3
rel_alt = 0
sleep_time = 15


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

def send_body_offset_ned_command(vx, vy):

    print("Sending Velocity commands!")
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
                        10, the_connection.target_system,
                        the_connection.target_component, 
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 
                        # int(0b110111111000), #pos
                        int(0b110111000111), #vel
                        0, 3, -10, 
                        vx, vy, 0, 
                        0, 0, 0, 
                        0, 0))

    while True:
        msg = the_connection.recv_match(
            type='NAV_CONTROLLER_OUTPUT', blocking=True)
        if msg:
            print(msg.wp_dist)
            if msg.wp_dist == 0:
                break

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



if (__name__ == "__main__"):
    # the_connection = mavutil.mavlink_connection('udpin:172.26.64.1:14550')
    the_connection = mavutil.mavlink_connection('COM3', 57600)

    await_heartbeat()

    arm()

    the_connection.motors_armed_wait()

    set_flight_mode('GUIDED')

    while True:
        msg = the_connection.recv_match(type ='HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)

            if "GUIDED" in mode:
                print("Taking off")
                takeoff(takeoff_height)
                break

    while True:
        msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
        if msg:
            print(msg.relative_alt)
            if msg.relative_alt >= ((takeoff_height * 1000) - 700): #20 centimeters shy of take off height
                send_body_offset_ned_command(velocity, 0)
                time.sleep(sleep_time)
                set_flight_mode("RTL")
                break


    cap = cv2.VideoCapture(1)

    centerX, centerY = w//2, h//2  #Get center of Frame
    
    # intrinsic_camera = np.array(((933.15867, 0, 657.59),(0,933.1586, 400.36993),(0,0,1)))
    # distortion = np.array((-0.43948,0.18514,0,0))
    
    intrinsic_camera = np.array(((534.9681530430366, 0, 305.39166889960603),(0,713.1386199148463, 255.72950392809662),(0,0,1)))
    distortion = np.array((-0.16161169421208624,
                            0.24648910664734525,
                            0.004137344187593899,
                            -0.0016340908890220367,
                            -0.43766632091131))
    
    horizontal_res = 640
    vertical_res = 480

    horizontal_fov = 90 * (math.pi / 180 )
    vertical_fov = 90 * (math.pi / 180)

    rand = random.randint(0, 1000)
    debug_image_writer = cv2.VideoWriter(f"vid_record_" + str(rand) + ".avi",cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 25.0,(w,h))

    while True:
        
        # frame = me.get_frame_read().frame
        ret, frame = cap.read()

        frame = cv2.resize(frame, (w,h))

        msg = the_connection.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
        if msg:
            rel_alt = msg.relative_alt
        cv2.putText(frame, f"Takeoff Alt: {takeoff_height} Relative Alt: {int(rel_alt/1000)}",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 255), 2)  
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        
        drawQuadrant()

        #Find aruCo marker in frame
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        parameters =  cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        # print(corners)
        if len(corners) > 0:   #if All 4 corners of marker is detected

            ids = ids.flatten()
            markerInfo = drawMarker(corners, ids)  

            cX, cY = markerInfo[0]   #Get marker midpoint
            
            rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.02,intrinsic_camera,distortion)
            
            cv2.aruco.drawDetectedMarkers(frame, corners) 

            try:
                cv2.drawFrameAxes(frame,intrinsic_camera,
                                distortion, rvec, tvec, 0.01)
            except Exception as e:
                continue
            
            
            x = tvec[0][0][0]
            y = tvec[0][0][1]
            z = tvec[0][0][2]
            
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = round((x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res), 2)
            y_ang =round((y_avg - vertical_res*.5)*(vertical_fov/vertical_res), 2)
            
            
            
            print(x_ang, y_ang)
            

            #Calculate error i.e Distance btw aruco midpoint and frame midpoint
            error = int(distance((cX, cY), (centerX, centerY)))
            markerWidth, markerHeight = markerInfo[1]
            
            cv2.putText(frame, f"X_vel: {x_ang}  Y_vel: {y_ang}  Error: {error}",
                    (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

            rW, rH = round(markerWidth/w, 1), round(markerHeight/h, 1)    #Calculate ratio
            # rW, rH = 0.4, 0.4
            #Draw line between center of frame and center of marker
            cv2.line(frame, (cX, cY), (centerX,centerY), (254, 255, 0), 3)


        cv2.imshow('video',frame)
        debug_image_writer.write(frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # me.land()
            # print(me.get_battery())
            break

# pip uninstall opencv-contrib-python

# 4.6.0.66
