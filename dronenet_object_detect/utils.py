import numpy as np
import cv2
# from cv2 import aruco
from time import sleep
import atexit
from djitellopy import tello
import winwifi
import os
import signal
import subprocess
import socket
from threading import Thread

import keyPressModule as kp
# import serial
import time



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

def simple_movement(distance):

    sleep(2)
    me.move_right(distance)
    sleep(2)

    me.move_left(distance - 50)
    # sleep(2)
    print("Searching for marker...")

def getKeyboardIinput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    global land

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
        me.streamoff()
    if kp.getKey("l"): 
        land = not land

    if kp.getKey("t"): 
        me.takeoff()
    return [lr, fb, ud, yv]

def triggerALRS(str):
    with serial.Serial('/dev/tty.usbserial-14330', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write(f'{str}'.encode('utf-8'))   

def triggerALRS(str):
    print(f"{str.upper()} ALRS Triggered ....................")
    print(f"{str.upper()} ALRS Triggered ....................")
    print(f"{str.upper()} ALRS Triggered ....................")
# simple_movement(300)

def alertCommandCenter():
    for i in range(2):  # send message twice; it's UDP!
        s.sendto(bytes("ALERT", "utf-8"), (controlCenterIPAddress, commandCenterPort))
        sleep(5)

def monitorUDP():
    while(1):
        data, address = s.recvfrom(20)
        if len(data) > 1:   # data will now be
            UDPDataAvailable = True # remember to make false whenever you've finished using
            UDPData = data
            print(UDPData)

def findCarOfInterest(img):
    lower_white = np.array([150, 150, 150])

    data = np.reshape(img, (-1,3))
    data = np.float32(data)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_RANDOM_CENTERS
    compactness,labels,centers = cv2.kmeans(data,1,None,criteria,10,flags)

    return centers[0].astype(np.int32) > lower_white

def findObjects(outputs, img):
    global informControlCenter
    hT, wT, cT = img.shape
    bbox = []
    classIds = []
    confs = []
    for output in outputs:
        for det in output:
            scores = det[5:]
            classId = np.argmax(scores)
            confidence = scores[classId]
            if confidence > confThreshold:
                w, h = int(det[2] * wT), int(det[3] * hT)
                x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                bbox.append([x, y, w, h])
                classIds.append(classId)
                confs.append(float(confidence))

    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)

    for i in indices:
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]

        car_img = img[int(y)-15:int(h)+15, int(x)-15:int(w)+15, ::-1]
        # isCarOfInterest = findCarOfInterest(car_img)

        # if isCarOfInterest and not informControlCenter:
        #     alertCommandCenter()
        #     informControlCenter = True

        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        print((classIds, classNames, i))
        if classIds[i]< 4:
            cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}%',
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

def handleUDPData(UDPData, distance):
    if UDPData == "LEFT":
        me.move_left(distance)
        sleep(1)
    elif UDPData == "RIGHT":
        me.move_right(distance)
        sleep(1)
    elif UDPData == "FRONT":
        me.move_forward(distance)
        sleep(1)
    elif UDPData == "BACK":
        me.move_back(distance)
        sleep(1)

def exitRoutine():
    print("will now try to kill ffmpeg")
    os.kill(ffmpegProcessId, signal.SIGTERM)
    s.close()
