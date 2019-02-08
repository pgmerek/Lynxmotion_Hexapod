import sys
sys.path.append("../project_files/robot_drivers/")

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

# Setup robot
hex_walker = Hex_Walker(rf, rm, rr, lr, lm, lf)
hex_walker.do_move_set([TALL_NEUTRAL])

def main():
	print "Controls:"
	print "- W is forward"
	print "- A is strafe left"
	print "- S is backwards"
	print "- D is strafe right"
	print "- Q is rotate left"
	print "- E is rotate right"

	while(True):
		command = enter_control()
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



def enter_control():
	user_input = raw_input("W, A, S, D, Q, or E: ")
	if user_input == 'w' or user_input == 'W' \
	or user_input == 'a' or user_input == 'A' \
	or user_input == 's' or user_input == 'S' \
	or user_input == 'd' or user_input == 'D' \
	or user_input == 'q' or user_input == 'Q' \
	or user_input == 'e' or user_input == 'E':
		return user_input
	else:
		print "User entered {0}".format(user_input)
		return 'Do_nothing'


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
	print "Rotating {0}".format(direction)
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


if __name__ == "__main__":
	main()

