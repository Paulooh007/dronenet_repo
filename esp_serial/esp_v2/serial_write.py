import serial
import time

ser = serial.Serial('COM7', 9600) # replace 'COM1' with the name of your serial port
while True:
    data = 'Hello\n'
    ser.write(data.encode("utf-8"))
    time.sleep(2)