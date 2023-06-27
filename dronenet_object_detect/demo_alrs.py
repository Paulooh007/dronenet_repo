import numpy as np
import cv2
from cv2 import aruco
from time import sleep
import atexit
from djitellopy import tello

import keyPressModule as kp
import serial
import time


isLanding = False
isOpen = False

w, h = 1000, 680
land = False

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

def getQuadrantInfo(cX, cY, centerX, centerY):
    # """Return region/quadrant and ΔX and ΔY"""
    delX = abs(cX - centerX)
    delY = abs(cY - centerY)
    region = 0
    if cX <= centerX and cY <= centerY:
        region = 1
    elif cX >= centerX and cY <= centerY:
        region = 2
    elif cX >= centerX and cY >= centerY:
        region = 3
    elif cX <= centerX and cY >= centerY:
        region = 4

    return (region, (delX, delY))

def getDirection(speedX,speedY,region):
    """
    +ve speedX = Right
    -ve speedX = Left
    +ve speedY = front
    -ve speedY = Back
    """

    if region == 1:
        return (-speedX, speedY)
    elif region == 2:
        return (speedX, speedY)
    elif region == 3:
        return (speedX, -speedY)
    elif region == 4:
        return (-speedX, -speedY)

def getKeyboardIinput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    global land
    global isLanding

    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = speed
    elif kp.getKey("d"): yv = -speed

    if kp.getKey("z"): 
        me.land()
        triggerALRS("close")
        # me.streamoff()

    if kp.getKey("l"):
        land, isLanding = not land, False
        sleep(0.2)

    if kp.getKey("t"): 
        me.takeoff()

    if kp.getKey("c"): 
        triggerALRS("close")

    if kp.getKey("o"): 
        triggerALRS("open")

    return [lr, fb, ud, yv]

def triggerALRS(str):
    with serial.Serial('COM5', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write(f'{str}'.encode('utf-8'))

def exitRoutine():
    # me.land()
    print("Killing all process")
    triggerALRS("close")


if (__name__ == "__main__"):
    kp.init()

    me = tello.Tello()
    me.connect()
    me.streamon()
    print(me.get_battery())

    atexit.register(exitRoutine)

    while True:
        
        vals = getKeyboardIinput()

        frame = me.get_frame_read().frame
        height = me.get_height()
        if height > 1000:  #Safety feature to land if height > 8 meters.
            me.land()
            break

        frame = cv2.resize(frame, (w,h))

        cv2.putText(frame, f"Altitude: {height} isLanding: {isLanding} Land: {land} Mode: {'Landing'}",
                        (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)  
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
        
        if land:
            centerX, centerY = w//2, h//2  #Get center of Frame

            drawQuadrant()

            #Find aruCo marker in frame
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
            parameters =  aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # print(corners)
            if len(corners) > 0:   #if All 4 corners of marker is detected

                ids = ids.flatten()
                markerInfo = drawMarker(corners, ids)  

                cX, cY = markerInfo[0]   #Get marker midpoint

                #Calculate error i.e Distance btw aruco midpoint and frame midpoint
                error = int(distance((cX, cY), (centerX, centerY)))
                markerWidth, markerHeight = markerInfo[1]


                rW, rH = round(markerWidth/w, 1), round(markerHeight/h, 1)    #Calculate ratio
                # rW, rH = 0.4, 0.4
                #Draw line between center of frame and center of marker
                cv2.line(frame, (cX, cY), (centerX,centerY), (254, 255, 0), 3)

                region, delXandY = getQuadrantInfo(cX, cY, centerX, centerY)   #Get the quadrant/region where marker is in and ΔX and ΔY
                delX, delY = delXandY
                delX, delY = int(rW * delX), int(rH * delY)  #Multiple error with ratio

                speedX, speedY = int(np.clip(delX, -100, 100)), int(np.clip(delY, -100, 100))   #front/back and left/right command to send to drone.
                speedX, speedY = getDirection(speedX,speedY,region) if not None else (0,0)    #Add direction to command

                cv2.putText(frame, f"X: {speedX}  Y: {speedY}  Error: {error} Alt: {height}  isLanding: {isLanding}",
                    (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2)

                if not isLanding:
                    me.send_rc_control(speedX, speedY, 0, 0)

                if height < 60: #Close enought to the ground
                    me.land()
                    triggerALRS("close")
                    print(me.get_battery())
                
                if error < 25 or isLanding:
                    if not isOpen:
                        isOpen = True
                        triggerALRS("open")
                    if not isLanding:
                        isLanding = True
                    me.send_rc_control(0, 0, -15, 0)
                    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                
            else:
                print("No marker.........................")
                # isLanding = False
                me.send_rc_control(0, 0, 0, 0) #Stay put
                me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        else:
            me.send_rc_control(vals[0], vals[1], vals[2], vals[3])


        cv2.imshow('video',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            triggerALRS("close")
            print(me.get_battery())
            break

