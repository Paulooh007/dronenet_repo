import serial
import time
import numpy as np

import argparse


parser = argparse.ArgumentParser(description='channel')
parser.add_argument('-r', "--reset", type=int, default=0, help='reset value')


args = parser.parse_args()

stick2 = serial.Serial("COM7", 9600)
stick1 = serial.Serial("COM9", 9600)

def critically_damped_system(target_value, time_val):
    mass = 1
    damping_constant = 14
    omega = np.sqrt(damping_constant / (4 * mass))
    position = target_value * (1 - np.exp(-1 * omega * time_val)) ** 1

    return position

def send_exponential_to_serial(target_value, duration, serial_port):
    
    start_time = time.time()
    duration = duration
    while True:
        elapsed_time = time.time() - start_time
        
        if elapsed_time >= duration:
            break

        current_value = critically_damped_system(target_value, elapsed_time)

        serial_port.write(f"{127},{round(current_value, 2)}\n".encode("utf-8"))

        print(str(round(current_value, 2)))

        delay = 0.01
        time.sleep(delay)

def send_ramp_to_serial(serial_port, peak_value, peak_time, dir):

    abs_peak_value = abs(peak_value)

    # Calculate the increment per time step
    increment = abs_peak_value / peak_time
    sleep_time = peak_time / abs_peak_value

    # Send the ramp values
    value = 0
    start_time = time.time()
    while value <= abs_peak_value:
        if dir == "r":
            if peak_value >= 0: 
                send_cmd = f"{127 + round(value, 2)},{127}\n"
            else:
                send_cmd = f"{127 - round(value, 2)},{127}\n"
        elif dir == "p":
            if peak_value >= 0: 
                send_cmd = f"{127},{127 + round(value, 2)}\n"
            else:
                send_cmd = f"{127},{127 - round(value, 2)}\n"
            
        print(send_cmd, dir)
        serial_port.write(send_cmd.encode("utf-8"))  # Convert the value to a string and send it as bytes
        time.sleep(sleep_time)
        elapsed_time = time.time() - start_time
        value = increment * elapsed_time

def send_ramp_to_serial_throttle(peak_value, peak_time):

    # Calculate the increment per time step
    increment = peak_value / peak_time
    sleep_time = peak_time / peak_value

    # Send the ramp values
    value = 0
    start_time = time.time()

    while value <= peak_value:

        send_cmd = f"{127},{round(value, 2)}\n"
        print(send_cmd, "t")

        stick1.write(send_cmd.encode("utf-8"))
        stick2.write(f"{127},{127}\n".encode("utf-8"))

        time.sleep(sleep_time)
        elapsed_time = time.time() - start_time
        value = increment * elapsed_time

def throttle_yaw(direction, position, duration):
    yaw = mid_position = 127
    throttle = 0

    if direction == "y":
        yaw = int((position/100) * 255)
    else:
        throttle = int((position/100) * 255)

    # send_exponential_to_serial(throttle, duration, stick1)
    send_ramp_to_serial_throttle(throttle, duration)
    
    # stick1.write(f"{yaw},{throttle}\n".encode("utf-8"))
    stick2.write(f"{mid_position},{mid_position}\n".encode("utf-8"))

def row_pitch(direction, position, duration):
    mid_position = 127
    roll = pitch = mid_position

    if direction == "r":
        roll = int((position/100) * 255) - 127
        send_ramp_to_serial(stick2, roll, duration, "r")
        
    else:
        pitch = int((position/100) * 255) - 127
        send_ramp_to_serial(stick2, pitch, duration, "p")

    stick2.write(f"{mid_position},{mid_position}\n".encode("utf-8")) # reset to mid position
    time.sleep(0.5)


def fly_path(paths):
    for dir in paths:
        direction, position, duration = dir

        if direction in ("y", "t"):
            throttle_yaw(direction, position, duration)
            time.sleep(1)
        elif direction in ("p", "r"):
            row_pitch(direction, position, duration)
            time.sleep(1)

def reset(throttle):
    stick1.write(f"{127},{throttle}\n".encode("utf-8"))
    stick2.write(f"{127},{127}\n".encode("utf-8"))




if __name__ == "__main__":

    if args.reset == 1:
        reset(0)
        print("Throttle is now reset to zero")
    else:
        reset(0)
        paths = [
            ("t", 60, 3), #takeoff
            ("p", 80, 10),
            ("r", 80, 10), 
            ("p", 20, 5), 
            ("r", 20, 5), 
            # ("t", 20, 3), # land 
        ]

        fly_path(paths)

    
