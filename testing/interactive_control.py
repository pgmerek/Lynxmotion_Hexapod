import sys
sys.path.append("../project_files/robot_drivers/")

import Adafruit_PCA9685
import time
from hex_walker_driver import *


# Declare globals
global right_side
global left_side
global hex_walker

def setup():
    #init the pwm stuffs and run selected tests
    global right_side
    global left_side
    global hex_walker

    right_side = Adafruit_PCA9685.PCA9685(address=0x40)
    left_side = Adafruit_PCA9685.PCA9685(address=0x41)
    # create some legs
    right_side.set_pwm_freq(60)
    left_side.set_pwm_freq(60)
    rf = Leg(0, right_side, 0, 1, 2, 0)
    rm = Leg(1, right_side, 3, 4, 5, 1)
    rr = Leg(2, right_side, 6, 7, 8, 2)
    lr = Leg(3, left_side, 0, 1, 2, 3)
    lm = Leg(4, left_side, 6, 4, 5, 4)
    lf = Leg(5, left_side, 3, 7, 8, 5)
    right_legs = [rf, rm, rr]
    left_legs = [lr, lm, lf]

    # Setup robot
    hex_walker = Hex_Walker(rf, rm, rr, lr, lm, lf)
    hex_walker.do_move_set([TALL_NEUTRAL])


def command_arbiter(command):
    if command == 'w' or command == 'W':
        tall_walk_test(hex_walker, 'forward')
    elif command == 'a' or command == 'A':
        tall_side_walk_test(hex_walker, 'left')
    elif command == 's' or command == 'S':
        tall_walk_test(hex_walker, 'backwards')
    elif command == 'd' or command == 'D':
        tall_side_walk_test(hex_walker, 'right')
    elif command == 'q' or command == 'Q':
        tall_rotate_test(hex_walker, 'left')
    elif command == 'e' or command == 'E':
        tall_rotate_test(hex_walker, 'right')


def tall_side_walk_test(hw, direction):
    hw.speed = 0.1
    moves = [
    TALL_NEUTRAL,
    TALL_TRI_FRONT_CENTER_UP_OUT_BACK_NEUTRAL,
    TALL_TRI_FRONT_CENTER_OUT_BACK_UP_NEUTRAL,
    TALL_TRI_FRONT_BACKWARDS_BACK_UP_NEUTRAL,
    TALL_TRI_FRONT_BACKWARDS_BACK_NEUTRAL,
    TALL_TRI_FRONT_UP_NEUTRAL_BACK_NEUTRAL,
    TALL_TRI_FRONT_UP_NEUTRAL_BACK_BACKWARDS,
    TALL_TRI_FRONT_NEUTRAL_BACK_BACKWARDS,
    TALL_TRI_FRONT_NEUTRAL_BACK_UP_NEUTRAL,
    TALL_NEUTRAL
    ]

    if direction == 'left':
        hw.set_new_front("3-4")
    else: 
        hw.set_new_front("0-1")

    hw.do_move_set(moves)
    hw.set_new_front("5-0")


def tall_walk_test(hw, direction):
    hw.speed = 0.05
    moves = [
    TALL_NEUTRAL,
    TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
    TALL_TRI_RIGHT_BACK_LEFT_FORWARD,
    TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
    TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
    TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
    TALL_TRI_RIGHT_FORWARD_LEFT_BACK,
    TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
    TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    TALL_NEUTRAL
    ]

    if direction == 'forward':
        hw.do_move_set(moves)
    else:
        hw.do_move_set(moves[::-1])



def tall_rotate_test(hw, direction):
    hw.speed = 0.1
    moves = [
    TALL_NEUTRAL,
    TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
    TALL_TRI_RIGHT_RIGHT_LEFT_LEFT,
    TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
    TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
    TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
    TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
    TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
    TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    TALL_NEUTRAL
    ]

    if direction == 'left':
        hw.do_move_set(moves)
    else:
        hw.do_move_set(moves[::-1])

