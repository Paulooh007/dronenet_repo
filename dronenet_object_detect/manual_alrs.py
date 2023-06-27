import serial
import time

# ls /dev/*

def triggerALRS(str):
    # with serial.Serial('/dev/tty.usbserial-14330', 9600, timeout=1) as ser:
    with serial.Serial('COM5', 9600, timeout=1) as ser:
        time.sleep(2)
        ser.write(f'{str}'.encode('utf-8'))


message = "close"
triggerALRS(message)
print(message)