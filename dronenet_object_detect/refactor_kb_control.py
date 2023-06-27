import cv2 as cv
import keyPressModule as kp
from djitellopy import tello
import socket
from threading import Thread
import os

from multiprocessing import Process
from time import sleep

nodeControllerIPAddress = "192.168.0.119"
controlCenterIPAddress = "192.168.0.128"

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

def getControlCenterInput(udpData):
    if udpData is None:
        return [0, 0, 0, 0]
    
    lr, fb, ud, yv = 0,0,0,0
    speed = 40

    if  udpData == "LEFT": lr = -speed
    elif udpData == "RIGHT": lr = speed

    if udpData == "UP": fb = -speed
    elif udpData == "DOWN": fb = speed

    if udpData == "w": ud = -speed
    elif udpData == "s": ud = speed

    if udpData == "a": yv = speed
    elif udpData == "d": yv = -speed

    if udpData == "z": me.land()
    if udpData == "t": me.takeoff()

    return [lr, fb, ud, yv]


message = '{ "event":"available_drones", \
            "data":{ "drones":[{ "name":"Drone 1",\
                 "feed_source":"192.168.0.119:4000" }] } }'

pkt_size = 188
buffer_size = 10000
max_delay = 1000

abstractCommand =  f"ffmpeg \
                    -i udp://{telloIPAddress}:{telloVideoSourcePort} \
                    -preset ultrafast \
                    -tune zerolatency \
                    -listen 1 \
                    -f mp4 \
                    -movflags frag_keyframe+empty_moov \
                    {nodeControllerIPAddress}:4000 \
                    ?pkt_size={str(pkt_size)}&buffer_size={str(buffer_size)}&max_delay={str(max_delay)}"


if __name__ == '__main__':
    # try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((nodeControllerIPAddress, nodeControllerPort))

    alertCommandCenter(message)

    me = tello.Tello()
    me.connect()
    print(me.get_battery())

    me.streamon()

    while True:
        data, address = s.recvfrom(50)
        if data:
            udpData = data.decode()
        else:
            udpData = None
            data = None

        vals = getControlCenterInput(udpData)
        me.send_rc_control(vals[0], vals[1], vals[2], vals[3])
        print(udpData, vals)

        udpData = None

    # except KeyboardInterrupt:
    #     print("Keyboard interrupt detected. Exiting...")
    #     me.land()  # add any necessary cleanup code here
    #     os._exit(0)  # force exit without cleanup to avoid potential hangs



# python dronenet_object_detection/keyboard_control.py 
# python dronenet_object_detection/refactor_kb_control.py 