import serial
import argparse
import time

parser = argparse.ArgumentParser(description='Serial Port Configuration')

# add arguments for port and command
parser.add_argument('-v', "--value", type=int, default=250, help='value')
parser.add_argument("-c","--channel", type=str, default='t', help='channel')
parser.add_argument("-m","--mode", type=str, default='manual', help='channel')

# left (yaw, throttle)

def set_string(channel, value):
    t, y, r, p = 127, 127, 127, 127
    # t, y, r, p = 0, 0, 0, 0
    if channel == "t":
        t = value
    elif channel == "y":
        y = value
    elif channel == "r":
        r = value
    elif channel == "p":
        p = value
    return t, y, r, p

def reset():
    # ser7.write(f"{0},{127}\n".encode("utf-8"))
    # ser9.write(f"{127},{127}\n".encode("utf-8"))
    ser7.write(f"{127},{127}\n".encode("utf-8"))
    ser9.write(f"{127},{127}\n".encode("utf-8"))

    


# parse the arguments
args = parser.parse_args()


if __name__  == "__main__":
    ser7 = serial.Serial("COM7", 9600)
    ser9 = serial.Serial("COM9", 9600)

    if args.mode == "manual":
        t, y, r, p = set_string(args.channel, args.value)

        ty = f"{y},{t}\n"
        pr = f"{r},{p}\n"

        print("ty", ty)
        print("pr", pr)

        ser9.write(ty.encode("utf-8"))
        ser7.write(pr.encode("utf-8"))


        # ser9.write(f"{yaw},{throttle}\n".encode())
        # ser7.write(f"{roll},{pitch}\n".encode())

    else:
        
        ser9.write(f"{127},{200}\n".encode("utf-8"))
        ser7.write(f"{127},{127}\n".encode("utf-8"))
        print("up")
        time.sleep(3)
        ser7.write(f"{127},{250}\n".encode("utf-8"))
        print("front")
        time.sleep(3)
        ser7.write(f"{250},{127}\n".encode("utf-8"))
        print("left")
        time.sleep(3)
        ser7.write(f"{127},{0}\n".encode("utf-8"))
        print("back")
        time.sleep(3)
        ser7.write(f"{127},{127}\n".encode("utf-8"))
        ser9.write(f"{127},{50}\n".encode("utf-8"))
        # time.sleep(5)
        # print("up")
        # time.sleep(2)
        # ser7.write(f"{127},{127}\n".encode("utf-8"))
        # print("front")
        # time.sleep(3)
        # ser9.write(f"{127},{80}\n".encode("utf-8"))   
        # print("down")

    # reset()       
    
