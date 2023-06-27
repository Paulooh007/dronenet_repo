import numpy as np
import time

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
        # current_value = target_value * (1 - math.exp(-elapsed_time / duration))

        serial_port.write(str(round(current_value, 2)).encode())
        print(str(round(current_value, 2)))

        delay = 0.01
        time.sleep(delay)


send_exponential_to_serial(100, 3)