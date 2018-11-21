import sys
sys.path.append("../robot_drivers/")

import Adafruit_PCA9685
import time
from hex_walker_driver import *

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
lm = Leg(0, pwm_41, 6, 4, 5, 4)
lf = Leg(0, pwm_41, 3, 7, 8, 5)

# create the hex walker
hex_walker = Hex_Walker(rf, rm, rr, lr, lm, lf)

# create the torso
r = Leg(0, pwm_41, 12, 11, 10, ARM_R)
l = Leg(0, pwm_40, 12, 11, 10, ARM_L)
rot = Rotator(0, pwm_40, 9)

torso = Robot_Torso(r, l, rot)

cmd = 999

while(cmd != 0):
    print("0 - quit")
    print("1 - torso does monkey")
    print("2 - torso waves")
    print("3 - walk 1 step")
    print("4 - walk 4 steps")
    print("5 - walk back 1 step")
    print("6 - walk back 4 step")
    print("7 - walk 120 degrees 1 step")
    print("8 - walk 120 degrees 4 steps")
    print("9 - rotate left 1 step")
    print("10 - rotate left 4 steps")
    print("11 - rotate right 1 step")
    print("12 - rotate right 4 steps")
    print("13 - hand shake")
    print("14 - king kong")
    print("15 - leg wave left")
    print("16 - leg wave right")
    print("17 - bounce down")

    cmd = input("Choose an option:")

    if(cmd == 1):
        print("doing the monkey 5 times")
        torso.monkey(5)
    elif(cmd == 2):
        print("waving to the left 3 times")
        torso.wave(45, 3)
    elif(cmd == 3):
        print("walking one step forward")
        hex_walker.walk(1, 0)
    elif(cmd == 4):
        print("walking 4 steps forward")
        hex_walker.walk(4,0)
    elif(cmd == 5):
        print("walking 1 step backward")
        hex_walker.walk(1, 180)
    elif(cmd == 6):
        print("walking 4 steps backward")
        hex_walker.walk(4, 180)
    elif(cmd == 7):
        print("walking 1 step at 120 degrees")
        hex_walker.walk(1, 120)
    elif(cmd == 8):
        print("walking 4 steps at 120 degrees")
        hex_walker.walk(4, 120)
    elif(cmd == 9):
        print("rotate left l step")
        hex_walker.rotate(1, LEFT)
    elif(cmd ==10):
        print("rotate left 4 steps")
        hex_walker.rotate(4, LEFT)
    elif(cmd == 11):
        print("rotate right 1 step")
        hex_walker.rotate(1, RIGHT)
    elif(cmd == 12):
        print("roate right 4 step")
        hex_walker.rotate(4, RIGHT)
    elif(cmd == 13):
        print("hand shake")
        torso.hand_shake(90, 4)
    elif(cmd == 14):
        print("king kong")
        torso.king_kong(90, 4)
    elif(cmd == 15):
        print("leg wave left")
        hex_walker.leg_wave(LEFT, .1, 4)
    elif(cmd == 16):
        print("leg wave right")
        hex_walker.leg_wave(RIGHT, .1, 4)
    elif(cmd == 17):
        print("bounce down")
        hex_walker.bounce(.3, 4)
