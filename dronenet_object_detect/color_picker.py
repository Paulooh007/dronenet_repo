import numpy as np
import cv2
from cv2 import aruco
from time import sleep
from djitellopy import tello
import time

import keyPressModule as kp


whT = 320
confThreshold = 0.3
nmsThreshold = 0.2
#### LOAD MODEL
classList = ["person","car","motorbike", "bus","truck"]
classNames = open('coco.names').read().strip().split('\n')
## Model Files
modelConfiguration = "yolov3-tiny.cfg"
modelWeights = "yolov3-tiny.weights"

modelConfiguration = "yolov3.cfg"
modelWeights = "yolov3.weights"
net = cv2.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)


w, h = 1000, 680

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

    if kp.getKey("t"): 
        me.takeoff()
    return [lr, fb, ud, yv]


def findCarOfInterest(img, color):
    lower_white = np.array([150, 150, 150])

    data = np.reshape(img, (-1,3))
    data = np.float32(data)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    flags = cv2.KMEANS_RANDOM_CENTERS
    _ ,_ ,centers = cv2.kmeans(data,1,None,criteria,10,flags)

    return centers[0].astype(np.int32) > lower_white

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

    indices = cv2.dnn.NMSBoxes(bbox, confs, confThreshold, nmsThreshold)

    for i in indices:
        box = bbox[i]
        x, y, w, h = box[0], box[1], box[2], box[3]

        if classNames[classIds[i]] in classList:

            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
            # print((classIds, classNames, i))

            cv2.putText(img, f'{classNames[classIds[i]].upper()} {int(confs[i] * 100)}%',
                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            if classNames[classIds[i]] == "car" or classNames[classIds[i]] == "bus" or classNames[classIds[i]] == "person":
                car_img = img[int(y)-15:int(h)+15, int(x)-15:int(w)+15, ::-1]
                
    
                try:
                    car_img = cv2.cvtColor(car_img, cv2.COLOR_BGR2RGB)
                    cv2.imwrite(f"./images/{classNames[classIds[i]]}_{str(time.time())}.jpg", car_img)
                except:
                    print("No object found")


if (__name__ == "__main__"):
    me = tello.Tello()
    me.connect()
    me.streamon()

    print(me.get_battery())



    kp.init()

    while True:
        
        vals = getKeyboardIinput()

        frame = me.get_frame_read().frame
        height = me.get_height()
        if height > 1000:  #Safety feature to land if height > 8 meters.
            me.land()
            break

        frame = cv2.resize(frame, (w,h))

        cv2.putText(frame, f"Altitude: {height}",
                        (20, 20), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 255), 1)

        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        print(vals[0], vals[1], vals[2], vals[3])

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

