import numpy as np
import cv2
from cv2 import aruco
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
import serial
import time

myIPAddress = "192.168.10.1"

ffmpegProcessId = 1000000
nodeControllerIPAddress = "192.168.43.179"
controlCenterIPAddress = "192.168.137.1"
controlCenterIPAddress = "192.168.0.108"

telloIPAddress = "0.0.0.0"
commandCenterPort = 33333
nodeControllerPort = 22222
telloVideoSourcePort = 11111
localVideoStreamPort = 4000
remoteVideoStreamPort = 5000

UDPDataAvailable = False
UDPData = ""
informControlCenter = False


whT = 320
confThreshold = 0.3
nmsThreshold = 0.2
#### LOAD MODEL
classNames = ["person","car","motorbike", "bus","truck"]
classNames = open('coco.names').read().strip().split('\n')
## Model Files
modelConfiguration = "yolov3-tiny.cfg"
modelWeights = "yolov3-tiny.weights"
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)


landingMode = False
isLanding = False
isOpen = False

w, h = 1000, 680
# w, h = 680, 480
land = False

telloCommand = "ffmpeg -probesize 32 -analyzeduration 0 -i udp://"\
          + str(telloIPAddress) + ":" \
          + str(telloVideoSourcePort) \
          + " -b 300k -minrate 200k -maxrate 400k -bufsize 600k -f mpegts udp://127.0.0.1:" \
          + str(localVideoStreamPort) \
          + "?pkt_size=188&buffer_size=8192 -f mpegts udp://" \
          + controlCenterIPAddress + ":" \
          + str(remoteVideoStreamPort) \
          + "?pkt_size=188&buffer_size=8192"

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
    global landingMode
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
        me.streamoff()

    if kp.getKey("l"): 
        land, isLanding = not land, False
        sleep(0.2)

    if kp.getKey("m"): 
        landingMode = not landingMode
        sleep(0.2)

    if kp.getKey("t"): 
        me.takeoff()
    return [lr, fb, ud, yv]

# def triggerALRS(str):
#     with serial.Serial('/dev/tty.usbserial-14330', 9600, timeout=1) as ser:
#         time.sleep(2)
#         ser.write(f'{str}'.encode('utf-8'))   

def triggerALRS(str):
    with serial.Serial('COM5', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write(f'{str}'.encode('utf-8'))

def triggerALRS(str):
    print("Print ALRS ..........................")
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
        # if classIds[i]< 4:
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
    # me.land()
    print("Killing all process")
    # triggerALRS("close")
    # print("will now try to kill ffmpeg")
    # os.kill(ffmpegProcessId, signal.SIGTERM)
    # s.close()


# 1
# s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# s.bind((myIPAddress, nodeControllerPort))

# telloCommand = "ffmpeg -probesize 32 -analyzeduration 0 -i udp://"\
#           + str(telloIPAddress) + ":" \
#           + str(telloVideoSourcePort) \
#           + " -f mpegts udp://127.0.0.1:" \
#           + str(localVideoStreamPort) \
#         #   + "?pkt_size=3000&buffer_size=1000&max_delay=1000"

if (__name__ == "__main__"):
    me = tello.Tello()
    me.connect()
    me.streamon()
    print(me.get_battery())

    atexit.register(exitRoutine)

    # 3
    try:
        command = telloCommand
        print(command)

        pro = subprocess.Popen(command,shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        ffmpegProcessId = pro.pid

        print(f"ffmpeg successful  ProcessId {ffmpegProcessId}")

    except:
        print("ffmpeg failed!")
        exit()

    # 2
    # udpListenerThread = Thread(target=monitorUDP)
    # udpListenerThread.start()
    # alertCommandCenter()


    kp.init()
    # localControl = True if (UDPData == "" or UDPData == "OVERIDEOVER") else False
    
    cap = cv2.VideoCapture(f"udp://127.0.0.1:{localVideoStreamPort}", cv2.CAP_FFMPEG)

    while True:
        
        localControl = True if (UDPData == "" or UDPData == "OVERIDEOVER") else False

        vals = getKeyboardIinput()

        # frame = me.get_frame_read().frame
        ret, frame = cap.read()
        height = me.get_height()
        if height > 1000:  #Safety feature to land if height > 8 meters.
            me.land()
            break

        frame = cv2.resize(frame, (w,h))

        cv2.putText(frame, f"Alt: {height} isLanding: {isLanding} Land: {land} Mode: {landingMode}",
                        (20, 150), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2)
        
        if landingMode:    
            
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

                    cv2.putText(frame, f"X: {speedX}  Y: {speedY}  Error: {error} Alt: {height}  isLanding: {isLanding} Mode: {landingMode}",
                        (20, 50), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2)
                    print( {"X": speedX , "Y": speedY})

                    if not isLanding:
                        me.send_rc_control(speedX, speedY, 0, 0)

                    if height < 10: #Close enought to the ground
                        me.land()
                        triggerALRS("close")
                        print(me.get_battery())
                    
                    if error < 10 or isLanding:
                        if not isOpen:
                            isOpen = True
                            triggerALRS("open")
                            # time.sleep(20)
                        if not isLanding:
                            isLanding = True
                        me.send_rc_control(0, 0, -20, 0) #halt
                    
                else:
                    print("No marker.........................")
                    # isLanding = False
                    me.send_rc_control(0, 0, 0, 0) #Stay put
            else:
                me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

            # cv2.imshow('video',frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     triggerALRS("close")
            #     me.land()
            #     print(me.get_battery())
            #     break

        else: 
            if localControl: #using command from node controller PC
                # vals = getKeyboardIinput()
                me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                print(vals[0], vals[1], vals[2], vals[3])

            # #4
            # elif UDPDataAvailable: #Use movement command from control center pc
            #     handleUDPData(UDPData, 20)
            #     print(f"UDPData: ", {UDPData})

            # img = me.get_frame_read().frame
            # img = cv2.resize(img, (480, 360))

            blob = cv2.dnn.blobFromImage(frame, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
            net.setInput(blob)
            layersNames = net.getLayerNames()
            outputNames = [(layersNames[i - 1]) for i in net.getUnconnectedOutLayers()]
            outputs = net.forward(outputNames)
            findObjects(outputs, frame)


            # isEventOfInterest = findCarOfInterest(img)
            # if isEventOfInterest:
            #     alertCommandCenter()

        cv2.imshow('video',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            triggerALRS("close")
            print(me.get_battery())
            break

