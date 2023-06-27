import numpy as np
import cv2
from cv2 import aruco
from time import sleep
import keyPressModule as kp

from djitellopy import tello

import time


me = tello.Tello()
me.connect()
print(me.get_battery())

# me.takeoff()
kp.init()


me.streamon()

w, h = 1000, 680


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


def getKeyboardIinput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 40

    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = speed
    elif kp.getKey("DOWN"): fb = -speed

    if kp.getKey("w"): ud = speed
    elif kp.getKey("s"): ud = -speed

    if kp.getKey("a"): yv = speed
    elif kp.getKey("d"): yv = -speed

    if kp.getKey("l"): 
        me.land()
        me.streamoff()

    if kp.getKey("t"): 
        me.takeoff()

    return [lr, fb, ud, yv]


while(True):
    vals = getKeyboardIinput()
    
    frame = me.get_frame_read().frame

    height = me.get_height()
    if height > 1000:  #Safety feature to land if height > 8 meters.
        me.land()
        break

    frame = cv2.resize(frame, (w,h))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 

    #Find aruCo marker in frame
    # aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters =  aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # print(corners)
    if len(corners) > 0:   #if All 4 corners of marker is detected

        ids = ids.flatten()
        drawMarker(corners, ids)  
        
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    cv2.imshow('video',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        print(me.get_battery())
        break

cv2.destroyAllWindows()