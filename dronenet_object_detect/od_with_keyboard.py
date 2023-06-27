import cv2 as cv
import numpy as np
import keyPressModule as kp
from djitellopy import tello

import serial
import time
from cv2 import aruco



whT = 320
confThreshold = 0.3
nmsThreshold = 0.2

w, h = 1000, 680
land = False

#### LOAD MODEL
classNames = ["person","car","motorbike", "bus","truck"]

## Model Files
modelConfiguration = "yolov3-tiny.cfg"
modelWeights = "yolov3-tiny.weights"
net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)


## Aruco Land Functions
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
    """Return region/quadrant and ΔX and ΔY"""

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

def triggerALRS(str):
    with serial.Serial('/dev/tty.usbserial-14330', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write(f'{str}'.encode('utf-8'))  

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


    
## Object detection function
def findObjects(outputs, img):
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

    indices = cv.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)

    for i in indices:
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]

        cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}%',
                (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)


kp.init()

me = tello.Tello()
me.connect()
print(me.get_battery())

me.streamon()

while True:

    vals = getKeyboardIinput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

    img = me.get_frame_read().frame
    img = cv.resize(img, (480, 360))

    blob = cv.dnn.blobFromImage(img, 1 / 255, (whT, whT), [0, 0, 0], 1, crop=False)
    net.setInput(blob)
    layersNames = net.getLayerNames()
    outputNames = [(layersNames[i - 1]) for i in net.getUnconnectedOutLayers()]
    outputs = net.forward(outputNames)
    findObjects(outputs, img)

    cv.imshow('Image', img)
    if cv.waitKey(1) & 0xFF == ord('q'):
        # cv2.imwrite(f"frame_{random.randint(1,100)}.png",frame)
        break