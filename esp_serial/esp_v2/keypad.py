import pygame
import serial
import argparse


parser = argparse.ArgumentParser(description='channel')
parser.add_argument('-t', "--throttle", type=int, default=0, help='throttle value')

args = parser.parse_args()


# Initialize Pygame
pygame.init()

# Define the size of the window
size = [400, 300]
screen = pygame.display.set_mode(size)

# Define the font
font = pygame.font.Font(None, 25)

# Define the default values for the throttle, yaw, pitch, and roll
throttle = args.throttle
yaw = 127
pitch = 127
roll = 127

default_value = 127
# Define the increment/decrement step size for the throttle, yaw, pitch, and roll
step_size = 2

# Define the colors for the text
black = (0, 0, 0)
white = (255, 255, 255)

# Set up the game loop
done = False
clock = pygame.time.Clock()

# Define the key status dictionary
key_status = {}

# Set up the serial ports
ser7 = serial.Serial('COM7', 9600)
ser9 = serial.Serial('COM9', 9600)

hand_on_keys = False  # initialize the variable

while not done:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.KEYDOWN:
            key_status[event.key] = True
            hand_on_keys = True  # set hand_on_keys to True when a key is pressed
            if event.key == pygame.K_r:
                throttle = yaw = pitch = roll = default_value
        elif event.type == pygame.KEYUP:
            key_status[event.key] = False
            hand_on_keys = any(key_status.values())  # set hand_on_keys to True if any key is still pressed
    
    # Reset pitch, yaw, and roll to default_value when hand is off the keys
    if not hand_on_keys:
        pitch = yaw = roll = default_value
    
    # Update the values based on the key status dictionary
    if key_status.get(pygame.K_w):
        throttle += step_size
        if throttle > 255:
            throttle = 255
    if key_status.get(pygame.K_s):
        throttle -= step_size
        if throttle < 0:
            throttle = 0
    if key_status.get(pygame.K_UP):
        pitch += step_size
        if pitch > 255:
            pitch = 255
    if key_status.get(pygame.K_DOWN):
        pitch -= step_size
        if pitch < 0:
            pitch = 0
    if key_status.get(pygame.K_LEFT):
        roll -= step_size
        if roll < 0:
            roll = 0
    if key_status.get(pygame.K_RIGHT):
        roll += step_size
        if roll > 255:
            roll = 255
    if key_status.get(pygame.K_a):
        yaw -= step_size
        if yaw < 0:
            yaw = 0
    if key_status.get(pygame.K_d):
        yaw += step_size
        if yaw > 255:
            yaw = 255
    
    throttle = round(throttle, 2)
    yaw = round(yaw, 2)
    pitch = round(pitch, 2)
    roll = round(roll, 2)

    # Send the values to the serial ports
    ser9.write(f"{yaw},{throttle}\n".encode())
    ser7.write(f"{roll},{pitch}\n".encode())
    
    # Clear the screen
    screen.fill(white)
    
    # Draw the text
    text_throttle = font.render(f"Throttle: {throttle}", True, black)
    text_yaw = font.render(f"Yaw: {yaw}", True, black)
    text_pitch = font.render(f"Pitch: {pitch}", True, black)
    text_roll = font.render(f"Roll: {roll}", True, black)
    
    screen.blit(text_throttle, [10, 10])
    screen.blit(text_yaw, [10, 30])
    screen.blit(text_pitch, [10, 50])
    screen.blit(text_roll, [10, 70])
    
    # Update the screen
    pygame.display.flip()
    
    # Limit the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()

# source C:/Users/kayod/miniconda3/Scripts/activate base
