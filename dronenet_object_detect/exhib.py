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

ffmpegProcessId = 1000000
myIPAddress = "192.168.137.1"
commandCenterPort = 33333
nodeControllerPort = 22222
controlCenterIPAddress = "192.168.137.59"
videoStreamPort = "5000"
UDPDataAvailable = False
UDPData = ""
informControlCenter = False


whT = 320
confThreshold = 0.3
nmsThreshold = 0.2
#### LOAD MODEL
classNames = ["person","car","motorbike", "bus","truck"]
## Model Files
modelConfiguration = "yolov3-tiny.cfg"
modelWeights = "yolov3-tiny.weights"
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

initiateLanding = False

# s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# s.bind((myIPAddress, nodeControllerPort))


# def exitRoutine():
#     print("will now try to kill ffmpeg")
#     os.kill(ffmpegProcessId, signal.SIGTERM)
#     s.close()


def alertCommandCenter():
    for i in range(2):  # send message twice; it's UDP!
        s.sendto(bytes("ALERT", "utf-8"), (controlCenterIPAddress, commandCenterPort))


def monitorUDP():
    while(1):
        data, address = s.recvfrom(20)
        if len(data) > 1:   # data will now be
            UDPDataAvailable = True # remember to make false whenever you've finished using
            UDPData = data
            print(UDPData)

def getKeyboardIinput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50

    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("RIGHT"): lr = speed

    if kp.getKey("UP"): fb = -speed
    elif kp.getKey("DOWN"): fb = speed

    if kp.getKey("w"): ud = -speed
    elif kp.getKey("s"): ud = speed

    if kp.getKey("a"): yv = speed
    elif kp.getKey("d"): yv = -speed

    if kp.getKey("q"): me.land()
    if kp.getKey("t"): me.takeoff()


    return [lr, fb, ud, yv]


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




if (__name__ == "__main__"):
    # atexit.register(exitRoutine)

    # telloConnected = False
    # myConnections = winwifi.WinWiFi.get_connected_interfaces()
    # for i in myConnections:
    #     if i.ssid == 'TELLO-F27A1B':
    #         telloConnected = True
    
    # if not telloConnected:
    #     try:
    #         winwifi.WinWiFi.connect('TELLO-F27A1B')
    #         print("Connection succeeded!")
    #     except:
    #         print("Connection failed!")
    #         exit()
    
    me = tello.Tello()
    me.connect()
    
    print(me.get_battery())
    sleep(2)
    # me.streamoff()
    me.streamon()

    # try:
    #     command = "ffmpeg -probesize 32 -analyzeduration 0 -i abstract.mp4 -b 300k -minrate 200k -maxrate 400k -bufsize 600k -f mpegts udp://127.0.0.1:5000?pkt_size=188&buffer_size=8192 -f mpegts udp://192.168.137.59:5000?pkt_size=188&buffer_size=8192"# -f sdl \"Tello\""
    #     pro = subprocess.Popen(command,shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    #     ffmpegProcessId = pro.pid

    # except:
    #     print("ffmpeg failed!")
    #     exit()

    # udpListenerThread = Thread(target=monitorUDP)
    # udpListenerThread.start()
    # alertCommandCenter()


    kp.init()

    # me = tello.Tello()
    # me.connect()
    # me.streamon()

    localControl = True if (UDPData == "" or UDPData == "OVERIDEOVER") else False

    while True:
        if initiateLanding:
            pass

        else: 
            if localControl: #using command from node controller PC
                # vals = getKeyboardIinput()
                # me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
                vals = getKeyboardIinput()
                print(vals[0], vals[1], vals[2], vals[3])
            elif UDPDataAvailable: #Use movement command from control center pc
                handleUDPData(UDPData, 20)
                print(f"UDPData: ", {UDPData})

            img = me.get_frame_read().frame
            img = cv2.resize(img, (1000, 600))

            blob = cv2.dnn.blobFromImage(img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
            net.setInput(blob)
            layersNames = net.getLayerNames()
            outputNames = [(layersNames[i - 1]) for i in net.getUnconnectedOutLayers()]
            outputs = net.forward(outputNames)
            findObjects(outputs, img)


            # isEventOfInterest = findCarOfInterest(img)
            # if isEventOfInterest:
            #     alertCommandCenter()

            cv2.imshow('Image', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

