import numpy as np
import cv2
from cv2 import aruco
from time import sleep
import atexit
from djitellopy import tello
import os
import signal
import subprocess
import socket
from threading import Thread

import keyPressModule as kp
import time

myIPAddress = "192.168.10.1"
myIPAddress = "192.168.0.119"    #WIFI 2

ffmpegProcessId = 1000000
nodeControllerIPAddress = "192.168.43.179"

controlCenterIPAddress = "192.168.137.1"
controlCenterIPAddress = "192.168.0.156"
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
classList = ["person","car","motorbike", "bus"]
classNames = open('coco.names').read().strip().split('\n')
## Model Files
modelConfiguration = "yolov3-tiny.cfg"
modelWeights = "yolov3-tiny.weights"

# modelConfiguration = "yolov3.cfg"
# modelWeights = "yolov3.weights"
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

objectToFind = None

w, h = 1000, 680
# w, h = 500, 300
land = False

telloCommand = "ffmpeg -probesize 32 -analyzeduration 0 -i udp://"\
          + str(telloIPAddress) + ":" \
          + str(telloVideoSourcePort) \
          + " -b 300k -minrate 200k -maxrate 400k -bufsize 600k -f mpegts udp://127.0.0.1:" \
          + str(localVideoStreamPort) \
          + "?pkt_size=188&buffer_size=10000&max_delay=1000 -f mpegts udp://" \
          + controlCenterIPAddress + ":" \
          + str(remoteVideoStreamPort) \
          + "?pkt_size=188&buffer_size=10000&max_delay=1000"

def getKeyboardIinput():
    lr, fb, ud, yv = 0,0,0,0
    speed = 50
    global informControlCenter
    global objectToFind

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
        # me.streamoff()

    if kp.getKey("v"): 
        objectToFind = "CAR"
        sleep(0.2)
    if kp.getKey("b"): 
        objectToFind = "BIKE"
        sleep(0.2)
    if kp.getKey("n"): 
        objectToFind = "PEDESTRIAN"
        sleep(0.2)
    if kp.getKey("c"):
        informControlCenter = not informControlCenter
        if informControlCenter:
            alertCommandCenter(objectToFind)
        sleep(0.2)

    if kp.getKey("t"): 
        me.takeoff()

    return [lr, fb, ud, yv]
  

def alertCommandCenter(message):
    for i in range(2):  # send message twice; it's UDP!
        s.sendto(bytes(message, "utf-8"), (controlCenterIPAddress, commandCenterPort))
        sleep(1)

def monitorUDP():
    while(1):
        data, address = s.recvfrom(20)
        if len(data) > 1:   # data will now be
            UDPDataAvailable = True # remember to make false whenever you've finished using
            UDPData = data
            print(UDPData)


def findObjects(outputs, img):
    global informControlCenter
    global objectToFind

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

        if classNames[classIds[i]] in classList:

            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)

            cv2.putText(img, f"{classNames[classIds[i]].upper() if classNames[classIds[i]] != 'person' else 'PEDESTRIAN'} {int(confs[i] * 100)}%",
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

def exitRoutine():
    print("Killing all process")
    print("will now try to kill ffmpeg")
    os.kill(ffmpegProcessId, signal.SIGTERM)
    s.close()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((myIPAddress, nodeControllerPort))


if (__name__ == "__main__"):

    me = tello.Tello()
    me.connect()
    me.streamon()
    print(me.get_battery())

    atexit.register(exitRoutine)
    kp.init()

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
    
    cap = cv2.VideoCapture(f"udp://127.0.0.1:{localVideoStreamPort}", cv2.CAP_FFMPEG)

    while True:
        
        vals = getKeyboardIinput()

        ret, frame = cap.read()
        height = me.get_height()
        if height > 1200:  #Safety feature to land if height > 8 meters.
            me.land()
            break

        frame = cv2.resize(frame, (w,h))

        cv2.putText(frame, f"Altitude: {height} Searching for {objectToFind if objectToFind else 'No'}",
                        (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)

        if informControlCenter:
            cv2.putText(frame, f"Found Vehicle of Interest",
                        (20, 80), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (0, 0, 200), 2)  
            cv2.putText(frame, f"Alerted Control Center",
                        (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (0, 0, 200), 2)  
            cv2.putText(frame, f"Found {objectToFind}",
                        (20, 160), cv2.FONT_HERSHEY_SIMPLEX,
                        1.2, (0, 0, 200), 2)       
        

        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        blob = cv2.dnn.blobFromImage(frame, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
        net.setInput(blob)
        layersNames = net.getLayerNames()
        outputNames = [(layersNames[i - 1]) for i in net.getUnconnectedOutLayers()]
        outputs = net.forward(outputNames)
        findObjects(outputs, frame)


        cv2.imshow('video',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            me.land()
            print(me.get_battery())
            break

