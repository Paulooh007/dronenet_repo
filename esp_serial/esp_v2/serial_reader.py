import serial
import time

ser = serial.Serial('COM1', 9600) # replace 'COM1' with the name of your serial port
while True:
    data = ser.readline()
    print(data.decode('utf-8'))
    time.sleep(1) # assuming the data is in utf-8 format
