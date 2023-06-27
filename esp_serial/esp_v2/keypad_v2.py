import pygame
import argparse
import serial


parser = argparse.ArgumentParser(description='channel')
parser.add_argument('-t', "--throttle", type=int, default=0, help='throttle value')

# Initialize Pygame
pygame.init()

# Define the size of the window
size = [400, 300]
screen = pygame.display.set_mode(size)

# Define the font
font = pygame.font.Font(None, 25)

# Define the default values for the throttle, yaw, pitch, and roll
throttle = 0
yaw = 127
pitch = 127
roll = 127

# Define the increment/decrement step size for the throttle, yaw, pitch, and roll
step_size = 5

# Define the colors for the text
black = (0, 0, 0)
white = (255, 255, 255)

ser7 = serial.Serial('COM7', 9600)
ser9 = serial.Serial('COM9', 9600)


# Set up the game loop
done = False
clock = pygame.time.Clock()

while not done:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            done = True
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                yaw = pitch = roll = 127
                throttle = 0


            if event.key == pygame.K_w:
                throttle += step_size
                if throttle > 255:
                    throttle = 255
            elif event.key == pygame.K_s:
                throttle -= step_size
                if throttle < 0:
                    throttle = 0
            elif event.key == pygame.K_UP:
                pitch += step_size
                if pitch > 255:
                    pitch = 255
            elif event.key == pygame.K_DOWN:
                pitch -= step_size
                if pitch < 0:
                    pitch = 0
            elif event.key == pygame.K_LEFT:
                roll -= step_size
                if roll < 0:
                    roll = 0
            elif event.key == pygame.K_RIGHT:
                roll += step_size
                if roll > 255:
                    roll = 255
            elif event.key == pygame.K_a:
                yaw -= step_size
                if yaw < 0:
                    yaw = 0
            elif event.key == pygame.K_d:
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
