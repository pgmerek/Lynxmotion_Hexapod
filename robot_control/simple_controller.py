import Adafruit_PCA9685
import time
from hex_walker_driver import *
import logging
import threading
import sys
sys.path.append('../robot_vision')
from object_detection import *


logging.basicConfig(level=logging.WARNING,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )

def torso_command(torso, command, args):
    if(command == "king_kong"):
        logging.warning("king_kong")
        torso.king_kong(args[0], args[1])
    elif(command == "monkey"):
        logging.warning("monkey")
        torso.monkey(args[0])

def walker_command(hex_walker, command, args):
    if(command == "bounce"):
        logging.warning("bounce")
        hex_walker.bounce(args[0], args[1])
    elif(command == "leg_wave"):
        logging.warning("leg_wave")
        hex_walker.leg_wave(args[0], args[1], args[2])
    elif(command == "walk"):
        logging.warning("walk")
        hex_walker.walk(args[0], args[1])

#init the pwm stuffs and run selected tests
pwm_40= Adafruit_PCA9685.PCA9685(address=0x40)
pwm_41= Adafruit_PCA9685.PCA9685(address=0x41)

pwm_40.set_pwm_freq(60)
pwm_41.set_pwm_freq(60)

# create somee legs
rf = Leg(0, pwm_40, 0, 1, 2, 0)
rm = Leg(0, pwm_40, 3, 4, 5, 1)
rr = Leg(0, pwm_40, 6, 7, 8, 2)
lr = Leg(0, pwm_41, 0, 1, 2, 3)
lm = Leg(0, pwm_41, 3, 4, 5, 4)
lf = Leg(0, pwm_41, 6, 7, 8, 5)

# create the hex walker
hex_walker = Hex_Walker(rf, rm, rr, lr, lm, lf)

# create the torso
r = Leg(0, pwm_41, 12, 11, 10, ARM_R)
l = Leg(0, pwm_40, 12, 11, 10, ARM_L)
rot = Rotator(0, pwm_40, 9)
torso = Robot_Torso(r, l, rot)

# colors are pink, blue, yellow, none
# sizes are small medium large, none
# locations are left right middle, none

camera = PiCamera ()
camera.resolution = RESOLUTION
camera.framerate = FRAME_RATE
raw_capture = PiRGBArray(camera, size=RESOLUTION)

time.sleep (0.1)
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    image = frame.array
    item = detect_color(image)
    # instead of open cv
    color = item.object_type # raw_input("object color:")
    size = item.size # raw_input("object size:")
    loc =  item.location # raw_input("object loc:")
    leg_cmd = "none"
    leg_args = []
    torso_cmd = "none"
    torso_args = []
    if color == "pink":
        leg_cmd = "bounce"
        leg_args = [.5, 4]
        torso_cmd = "king_kong"
        torso_args = [90, 4]
    elif color == "blue":
        leg_cmd = "leg_wave"
        leg_args = [LEFT, .1, 5]
        torso_cmd = "monkey"
        torso_args = [3]
    elif color == "yellow":
        if size == "small":
            leg_cmd = "walk"
            leg_args = [1, 0]
            torso_cmd = "none"
            torso_args = []
        elif size == "large":
            leg_cmd = "walk"
            leg_args = [1, 180]
            torso_cmd = "none"
            torso_args = []
    else:
        leg_cmd = "none"
        leg_args = []
        torso_cmd = "none"
        torso_args = []

    leg_thread = threading.Thread(name='leg_control', target=walker_command, args=(hex_walker,leg_cmd, leg_args))
    torso_thread = threading.Thread(name='torso_control', target=torso_command, args=(torso, torso_cmd, torso_args))
 
    leg_thread.start()
    torso_thread.start()
    leg_thread.join()
    torso_thread.join()
    raw_capture.truncate(0)




