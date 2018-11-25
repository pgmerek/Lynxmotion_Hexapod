import sys
sys.path.append("../robot_drivers/")

import Adafruit_PCA9685
import time
from hex_walker_driver import *

#init the pwm stuffs and run selected tests
right_side= Adafruit_PCA9685.PCA9685(address=0x40)
left_side= Adafruit_PCA9685.PCA9685(address=0x41)

# create some legs
right_side.set_pwm_freq(60)
left_side.set_pwm_freq(60)
sleep_time = 1
rf = Leg(0, right_side, 0, 1, 2, 0)
rm = Leg(1, right_side, 3, 4, 5, 1)
rr = Leg(2, right_side, 6, 7, 8, 2)
lr = Leg(3, left_side, 0, 1, 2, 3)
lm = Leg(4, left_side, 6, 4, 5, 4)
lf = Leg(5, left_side, 3, 7, 8, 5)
right_legs = [rf, rm, rr]
left_legs = [lr, lm, lf]
all_legs = right_legs + left_legs


def enter_angle(prompt, previous_value):
    user_input = raw_input(prompt)
    if not user_input:  # If empty input, assume that the value is meant to be repeated
        return previous_value

    return int(user_input)


leg_number = input("Enter the leg number:")
tip_angle = input("Enter the tip angle:")
mid_angle = input("Enter the mid angle:")
rot_angle = input("Enter the rot angle:")
print("\n")
all_legs[leg_number].set_leg_position(Leg_Position(tip_angle, mid_angle, rot_angle))


while(True):
    leg_number = enter_angle("Enter the leg number:", leg_number)
    tip_angle = enter_angle("Enter the tip angle:", tip_angle)
    mid_angle = enter_angle("Enter the mid angle:", mid_angle)
    rot_angle = enter_angle("Enter the rot angle:", rot_angle)
    print("\n")
    all_legs[leg_number].set_leg_position(Leg_Position(tip_angle, mid_angle, rot_angle))

