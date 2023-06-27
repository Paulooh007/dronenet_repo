import cv2 as cv
import numpy as np
import keyPressModule as kp
from djitellopy import tello
import socket
from threading import Thread
import os
from multiprocessing import Process

from time import sleep

myIPAddress = "192.168.0.119"    #WIFI 2

nodeControllerIPAddress = "192.168.43.179"

controlCenterIPAddress = "192.168.0.128"
# controlCenterIPAddress = "192.168.0.108"

telloIPAddress = "0.0.0.0"
commandCenterPort = 33333
nodeControllerPort = 22222
telloVideoSourcePort = 11111
localVideoStreamPort = 4000
remoteVideoStreamPort = 5000

udpData = ""

def alertCommandCenter(message):
    for i in range(2):  # send message twice; it's UDP!
        s.sendto(bytes(message, "utf-8"), (controlCenterIPAddress, commandCenterPort))
        sleep(1)

def monitorUDP():
    while(1):
        data, address = s.recvfrom(50)
        sleep(0.5)
        if len(data) > 1:   # data will now be
            udpData = data.decode()
            print(udpData)

def getKeyboardInput():
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

    if kp.getKey("z"): me.land()
    if kp.getKey("t"): me.takeoff()

    return [lr, fb, ud, yv]

def getControlCenterInput(udpData):
    lr, fb, ud, yv = 0,0,0,0
    speed = 50

    if  udpData == "LEFT": lr = -speed
    elif udpData == "RIGHT": lr = speed

    if udpData == "UP": fb = -speed
    elif udpData == "DOWN": fb = speed

    if udpData == "w": ud = -speed
    elif udpData == "s": ud = speed

    if udpData == "a": yv = speed
    elif udpData == "d": yv = -speed

    # if udpData == "z": me.land()
    # if udpData == "t": me.takeoff()

    return [lr, fb, ud, yv]

message = '{ "event":"available_drones", \
            "data":{ "drones":[{ "name":"Drone 1",\
                 "feed_source":"192.168.0.119:4000" }] } }'



# abstractCommand = f"ffmpeg -probesize 32 -analyzeduration 0 -i udp://0.0.0.0:11111 -listen 1 -f mp4 -movflags frag_keyframe+empty_moov http://192.168.0.119:4000"

abstractCommand = f"ffmpeg -probesize 32 -analyzeduration 0 -i udp://0.0.0.0:11111 -b 300k -minrate 200k -maxrate 400k -bufsize 600k -listen 1 -f mp4 -movflags frag_keyframe+empty_moov http://192.168.0.119:4000"

abstractCommand =  "ffmpeg -i udp://0.0.0.0:11111 -preset ultrafast -tune zerolatency -listen 1 -f mp4 -movflags frag_keyframe+empty_moov http://192.168.0.119:4000?pkt_size=188&buffer_size=10000&max_delay=1000"

if __name__ == '__main__':

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((myIPAddress, nodeControllerPort))

    alertCommandCenter(message)

    
    # udpListener = Thread(target=monitorUDP)
    # udpListener.start()
    

# kp.init()

    me = tello.Tello()
    me.connect()
    print(me.get_battery())

    me.streamon()
    # os.system(abstractCommand)

    while True:

        data, address = s.recvfrom(50)
        if data:
            udpData = data.decode()
        else:
            udpData = None

        vals = getControlCenterInput(udpData)
        print(udpData, vals)
        # me.send_rc_control(vals[0], vals[1], vals[2], vals[3])

        udpData = None
        vals= [0,0,0,0]

        # img = me.get_frame_read().frame
        # img = cv.resize(img, (480, 360))

        # cv.imshow('Image', img)
        if cv.waitKey(1) & 0xFF == ord('q'):
            # cv2.imwrite(f"frame_{random.randint(1,100)}.png",frame)
            break
