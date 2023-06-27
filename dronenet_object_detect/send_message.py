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


ffmpegProcessId = 1000000
myIPAddress = "192.168.137.46"
# myIPAddress = "192.168.10.3"

commandCenterPort = 33333
nodeControllerPort = 22222
# controlCenterIPAddress = "192.168.137.59"
controlCenterIPAddress = "192.168.137.1"


videoStreamPort = "5000"
UDPDataAvailable = False
UDPData = ""
informControlCenter = False


def alertCommandCenter():
    for i in range(3):  # send message twice; it's UDP!
        s.sendto(bytes("ALERT", "utf-8"), (controlCenterIPAddress, commandCenterPort))
        sleep(5)

def monitorUDP():
    while(1):
        data, address = s.recvfrom(20)
        if len(data) > 1:   # data will now be
            UDPDataAvailable = True # remember to make false whenever you've finished using
            UDPData = data
            print(UDPData)


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



s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind((myIPAddress, nodeControllerPort))


if (__name__ == "__main__"):
    udpListenerThread = Thread(target=monitorUDP)
    udpListenerThread.start()
    alertCommandCenter()

    while True:
        alertCommandCenter()
 