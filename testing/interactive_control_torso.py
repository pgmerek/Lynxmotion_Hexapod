"""
Simple way to control the torso through a ui
Author: Patrick Gmerek
"""
import sys
sys.path.append("../robot_control/")

import Adafruit_PCA9685
import numpy as np
import cv2 as cv
import time
from robot_control.hex_walker_driver import *


def main():
    torso = initialize_torso()
    slider_names = ["Waist",
                    "Right Rotary Joint", "Right Mid Joint", "Right Tip Joint",
                    "Left Rotary Joint", "Left Mid Joint", "Left Tip Joint"]
    slider_limits = [[45, 135],
                     [0, 180], [0, 180], [0, 180],
                     [0, 180], [0, 180], [0, 180]]
    window_name = "Hexapod Torso Control"
    cv.namedWindow(window_name)

    for slider, limits in zip(slider_names, slider_limits):
        cv.createTrackbar(slider, window_name, limits[0], limits[1], dummy)

    user_inputs = fetch_trackbar_pos(window_name, slider_names)

    torso[0].set_leg_position((user_inputs[1], user_inputs[2], user_inputs[3]))
    torso[1].set_leg_position((user_inputs[1], user_inputs[2], user_inputs[3]))
    torso[2].set_angle(user_inputs[0])
    while True:
        previous_user_inputs = user_inputs
        user_inputs = fetch_trackbar_pos(window_name, slider_names)
        key = cv.waitKey(1) & 0xFF
        time.sleep(0.05)    # Our poor processor....
        if key == ord("q"):  # Quit if the user presses "q"
            break
        if not compare_lists(user_inputs, previous_user_inputs):
            print("Values changed")
            torso[0].set_leg_position((user_inputs[1], user_inputs[2], user_inputs[3]))
            torso[1].set_leg_position((user_inputs[1], user_inputs[2], user_inputs[3]))
            torso[2].set_angle(user_inputs[0])
    cv.destroyAllWindows()


def compare_lists(list1, list2):
    if not len(list1) == len(list1):
        return -1

    for i in range(0, len(list1)):
        if not list1[i] == list2[i]:
            return 0

    return 1


def fetch_trackbar_pos(window_name, slider_names):
    leg_num = cv.getTrackbarPos(slider_names[0], window_name)
    rot_angle = cv.getTrackbarPos(slider_names[1], window_name)
    mid_angle = cv.getTrackbarPos(slider_names[2], window_name)
    tip_angle = cv.getTrackbarPos(slider_names[3], window_name)
    return [leg_num, rot_angle, mid_angle, tip_angle]


def dummy(x):
    return


def initialize_torso():
    pwm_40= Adafruit_PCA9685.PCA9685(address=0x40)
    pwm_41= Adafruit_PCA9685.PCA9685(address=0x41)

    pwm_40.set_pwm_freq(60)
    pwm_41.set_pwm_freq(60)

    sleep_time = 2
    # create the torso
    r = Leg(0, pwm_41, 12, 11, 10, ARM_R)
    l = Leg(0, pwm_40, 12, 11, 10, ARM_L)
    rot = Rotator(0, pwm_40, 9)

    return [r, l, rot]


if __name__ == '__main__':
    main()
