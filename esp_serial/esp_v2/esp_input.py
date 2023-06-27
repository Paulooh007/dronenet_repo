import serial
import time
import random


def scale_down(value):
    scaled_value = (value / 250) * 3.3
    return scaled_value


value1 = 240 # generate a random number between 0 and 250
value2 = 240

ser = serial.Serial('COM7', 9600) # replace 'COM7' with the correct port name
output = f'"{value1},{value2}"'
ser.write(bytes(str(output), 'utf-8')) 
print(output)

print(f'{scale_down(value1)},{scale_down(value2)}')

# def encode(value1, value2):
#     start_delimiter = b'<'
#     end_delimiter = b'>'
#     separator = b','
#     encoded_value1 = bytes(str(value1), 'utf-8')
#     encoded_value2 = bytes(str(value2), 'utf-8')
#     encoded_message = start_delimiter + encoded_value1 + separator + encoded_value2 + end_delimiter
#     return encoded_message

# value1 = 78
# value2 = 100

ser = serial.Serial('COM7', 9600)
message = encode(value1, value2)
ser.write(message)


# while True:
#     value1 = random.randint(0, 250) # generate a random number between 0 and 250
#     value2 = random.randint(0, 250)
#     print(f'{scale_down(value1)},{scale_down(value2)}')
#     # print(value2, scale_down(value2))
#     ser.write(bytes(f'{value1},{value2}', 'utf-8')) # send the value to the serial port
#     time.sleep(4) # wait for 2 seconds before sending the next value
