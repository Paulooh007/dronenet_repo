import time
import math
import argparse
import sys
import csv

from dronekit import connect, VehicleMode
from pymavlink import mavutil

import cv2
import cv2 as aruco
import numpy as np


######VARIABLES####################
##Aruco
id_to_find = 72
marker_size = 19 #cm
takeoff_height = 8
velocity = .5

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()

##Camera
horizontal_res = 640
vertical_res = 480
cap = cv2.VideoCapture()

horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path="/home/pi/video2calibration/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')


##Counters and script triggers
found_count=0
notfound_count=0

first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
script_mode = 1 ##1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land=0 ##1 to trigger landing

manualArm=True ##If True, arming from RC controller, If False, arming from this script.


#########FUNCTIONS#################
def ack(keyword):
    print("-- Message Read " + \
          str(vehicle.recv_match(type=keyword, blocking=True))
          )
    
def connect_vehicle(connection_string, baud_rate=57600):
    vehicle = mavutil.mavlink_connection(connection_string, baud_rate)
	
    return vehicle

def arm(vehicle):
    print("*-- Arming*")

    vehicle.mav.command_long_send(
            vehicle.target_system, 
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,1,0,0,0,0,0,0
            )
    
    ack(vehicle, "COMMAND_ACK")

def takeoff(vehicle, height):
    print("*-- Takeoff Initiated*")

    vehicle.mav.command_long_send(
            vehicle.target_system, 
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,0,0,0,math.nan,0,0,height)
    
    ack(vehicle, "COMMAND_ACK")


def set_flight_mode(vehicle, fmode):
    mode = fmode

    # Check if mode is available
    if mode not in vehicle.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(vehicle.mode_mapping().keys()))
        sys.exit(1)

    # Get mode ID
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    


def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()
	

def send_body_offset_ned_command(vx, vy):

    # Create SET_POSITION_TARGET_LOCAL_NED message
    msg = vehicle.mav.set_position_target_local_ned_encode(
        0,                          # time_boot_ms (not used)
        vehicle.target_system,   # target system ID
        vehicle.target_component,  # target component ID
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000000000000111,         # type_mask (only consider velocity)
        0, 0, 0,                    # x, y, and z positions (NED frame)
        vx, vy, 0,                  # x, y, and z velocities (NED frame)
        0, 0, 0,                    # x, y, and z accelerations (NED frame)
        0, 0)                       # yaw, yaw_rate

    # Send the message
    vehicle.send(msg)



def get_mode():
    # Get the current flight mode
    mode = vehicle.mode_mapping()[vehicle.flightmode]

    # Print or return the flight mode
    print("Current Flight Mode: ", mode)
    return mode

def lander():
	global first_run,notfound_count,found_count,marker_size,start_time
	if first_run==0:
		print("First run of lander!!")
		first_run=1
		start_time=time.time()
		
	frame = cap.read()
	frame = cv2.resize(frame,(horizontal_res,vertical_res))
	frame_np = np.array(frame)
	gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)
	ids=''
	corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
	if get_mode() != 'LAND':
		set_flight_mode("LAND")
		while get_mode()!='LAND':
			print('WAITING FOR DRONE TO ENTER LAND MODE')
			time.sleep(1)
	try:
		if ids is not None and ids[0] == id_to_find:  #If marker is identified
			ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
			(rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])
			x = '{:.2f}'.format(tvec[0])
			y = '{:.2f}'.format(tvec[1])
			z = '{:.2f}'.format(tvec[2])
			
			y_sum = 0
			x_sum = 0
			
			x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
			y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
	
			x_avg = x_sum*.25
			y_avg = y_sum*.25
			
			x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
			y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
			
			if get_mode()!='LAND':
				set_flight_mode("LAND")
				while get_mode()!='LAND':
					time.sleep(1)
				print("------------------------")
				print("Vehicle now in LAND mode")
				print("------------------------")
				send_body_offset_ned_command(x_ang,y_ang)
			else:
				send_body_offset_ned_command(x_ang,y_ang)
				pass
			print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
			print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))  #No of times marker is found/not found in frame
			print("MARKER POSITION: x=" +x+" y= "+y+" z="+z)
			found_count=found_count+1
			print("")
		else:
			notfound_count=notfound_count+1
	except Exception as e:
		print('Target likely not found. Error: '+str(e))
		notfound_count=notfound_count+1




vehicle = connect_vehicle("COM7")


##
##SETUP PARAMETERS TO ENABLE PRECISION LANDING
##
# vehicle.parameters['PLND_ENABLED'] = 1
# vehicle.parameters['PLND_TYPE'] = 1 ##1 for companion computer
# vehicle.parameters['PLND_EST_TYPE'] = 0 ##0 for raw sensor, 1 for kalman filter pos estimation
# vehicle.parameters['LAND_SPEED'] = 20 ##Descent speed of 30cm/s


arm()

vehicle.motors_armed_wait()

set_flight_mode('GUIDED')

takeoff(10)

while True:
    msg = vehicle.recv_match(type = "GLOBAL_POSITION_INT", blocking = False)
    if msg:
        if msg.relative_alt >= ((takeoff_height * 1000)):  #20 centimeters shy of take off height
            # start_mission(the_connection)
            ready_to_land = True
            break

if ready_to_land:
	lander()
