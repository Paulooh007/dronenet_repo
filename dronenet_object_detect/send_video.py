import numpy as np
import cv2
from time import sleep
import atexit
from djitellopy import tello
import os
import signal
import subprocess
import socket
from threading import Thread


myIPAddress = "192.168.10.1"

ffmpegProcessId = 1000000
nodeControllerIPAddress = "192.168.43.179"
# controlCenterIPAddress = "192.168.137.1"
controlCenterIPAddress = "192.168.0.108"
controlCenterIPAddress = "192.168.0.108"
# controlCenterIPAddress = "192.168.137.1"

telloIPAddress = "0.0.0.0"
commandCenterPort = 33333
nodeControllerPort = 22222
telloVideoSourcePort = 11111
localVideoStreamPort = 4000
remoteVideoStreamPort = 5000

UDPDataAvailable = False
UDPData = ""
informControlCenter = False


def exitRoutine():
    print("will now try to kill ffmpeg")
    os.kill(ffmpegProcessId, signal.SIGTERM)


telloCommand = "ffmpeg -i udp://"\
          + str(telloIPAddress) + ":" \
          + str(telloVideoSourcePort) \
          + " -b 300k -minrate 200k -maxrate 400k -bufsize 600k -f mpegts udp://127.0.0.1:" \
          + str(localVideoStreamPort) \
          + "?pkt_size=376&buffer_size=8192 -f mpegts udp://" \
          + controlCenterIPAddress + ":" \
          + str(remoteVideoStreamPort) \
          + "?pkt_size=376&buffer_size=8192"



abstractCommand = "ffmpeg -probesize 32 -analyzeduration 0 -i abstract.mp4 -b 300k -minrate 200k -maxrate 400k -bufsize 600k -f mpegts udp://127.0.0.1:"\
                  +str(localVideoStreamPort)\
                  +"?pkt_size=188&buffer_size=8192 -f mpegts udp://"\
                  + controlCenterIPAddress\
                  +":"\
                  +str(remoteVideoStreamPort)\
                  + "?pkt_size=188&buffer_size=8192 -f sdl \"Tello\""

if (__name__ == "__main__"):
    # atexit.register(exitRoutine)


    me = tello.Tello()
    me.connect()
    me.streamon()
    print(me.get_battery())

    try:
        command = telloCommand
        print(command)

        pro = subprocess.Popen(command,shell=False, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        ffmpegProcessId = pro.pid

        print(f"ffmpeg successful  ProcessId {ffmpegProcessId}")

    except:
        print("ffmpeg failed!")
        exit()

    sleep(20)
    # atexit.register(exitRoutine)



 