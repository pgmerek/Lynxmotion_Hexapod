"""
Simple way to control each leg through a ui
Author: Patrick Gmerek
"""
import Adafruit_PCA9685
import numpy as np
import cv2 as cv
import time
from robot_control.hex_walker_driver import *


def main():
    all_legs = initialize_legs()
    slider_names = ["Leg", "Rotary Joint", "Mid Joint", "Tip Joint"]
    slider_limits = [[1, 5], [0, 180], [45, 180], [60, 180]]
    window_name = "Hexapod Leg Control"
    cv.namedWindow(window_name)

    for slider, limits in zip(slider_names, slider_limits):
        cv.createTrackbar(slider, window_name, limits[0], limits[1], dummy)

    user_inputs = fetch_trackbar_pos(window_name, slider_names)
    all_legs[user_inputs[0]].set_leg_position(Leg_Position(user_inputs[1], user_inputs[2], user_inputs[3]))
    while True:
        previous_user_inputs = user_inputs
        user_inputs = fetch_trackbar_pos(window_name, slider_names)
        key = cv.waitKey(1) & 0xFF
        time.sleep(0.05)    # Our poor processor....
        if key == ord("q"):  # Quit if the user presses "q"
            break
        if not compare_lists(user_inputs, previous_user_inputs):
            print("Values changed")
            all_legs[user_inputs[0]].set_leg_position(Leg_Position(user_inputs[1], user_inputs[2], user_inputs[3]))
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


def initialize_legs():
    # init the pwm stuffs and run selected tests
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
    lm = Leg(4, left_side, 3, 4, 5, 4)
    lf = Leg(5, left_side, 6, 7, 8, 5)
    right_legs = [rf, rm, rr]
    left_legs = [lr, lm, lf]
    all_legs = right_legs + left_legs

    return all_legs


if __name__ == '__main__':
    main()
