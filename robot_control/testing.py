import Adafruit_PCA9685
import time
from hex_walker_driver import *

print("working")
def helper_test():
  print("----------------------------helper function tests------------------")
  print("slope should return " + str(135./300.))
  print(str(slope(TIP_MOTOR_IN, TIP_MOTOR_IN_ANGLE, TIP_MOTOR_OUT, TIP_MOTOR_OUT_ANGLE)))
  print("intercept should return " + str(0-52))
  print(str(intercept(16, 12, 4)))
  print("should print out a nice-ish mapping")
  for i in range(0, 90):
    print("value: " +str(i) + " maps to " + str(linear_map(0, 200, 90, 300, i)))


def tip_motor_test(leg):
  print("--------------------------tip motor test starting---------------")
  print("tip motor to 0")
  leg.set_angle(0, TIP_MOTOR)
  time.sleep(sleep_time)
  print("tip motor to 45")
  leg.set_angle(45, TIP_MOTOR)
  time.sleep(sleep_time)
  print("tip motor to 90")
  leg.set_angle(90, TIP_MOTOR)
  time.sleep(sleep_time)
  print("tip motor to 135")
  leg.set_angle(135, TIP_MOTOR)
  time.sleep(sleep_time)
  print("tip motor to 180")
  leg.set_angle(180, TIP_MOTOR)
  time.sleep(sleep_time)
  print("tip motor to 270")
  leg.set_angle(270, TIP_MOTOR)
  time.sleep(sleep_time)

def mid_motor_test(leg):
  print("------------------------mid motor test starting-------------")
  print("mid motor to 0")
  leg.set_angle(0, MID_MOTOR)
  time.sleep(sleep_time)
  print("mid motor to 45")
  leg.set_angle(45, MID_MOTOR)
  time.sleep(sleep_time)
  print("mid motor to 90")
  leg.set_angle(90, MID_MOTOR)
  time.sleep(sleep_time)
  print("mid motor to 135")
  leg.set_angle(135, MID_MOTOR)
  time.sleep(sleep_time)
  print("mid motor to 180")
  leg.set_angle(180, MID_MOTOR)
  time.sleep(sleep_time)
  print("mid motor to 270")
  leg.set_angle(270, MID_MOTOR)
  time.sleep(sleep_time)

def rot_motor_test(leg):
  print("----------------------------rot motor test starting-----------------")
  print("rot motor to 0")
  leg.set_angle(0, ROT_MOTOR)
  time.sleep(sleep_time)
  print("rot motor to 45")
  leg.set_angle(45, ROT_MOTOR)
  time.sleep(sleep_time)
  print("rot motor to 90")
  leg.set_angle(90, ROT_MOTOR)
  time.sleep(sleep_time)
  print("rot motor to 135")
  leg.set_angle(135, ROT_MOTOR)
  time.sleep(sleep_time)
  print("rot motor to 180")
  leg.set_angle(180, ROT_MOTOR)
  time.sleep(sleep_time)
  print("rot motor to 270")
  leg.set_angle(270, ROT_MOTOR)
  time.sleep(sleep_time)

def neutral(leg):
  leg.set_angle(0, TIP_MOTOR)
  leg.set_angle(180, MID_MOTOR)
  leg.set_angle(90, ROT_MOTOR)

def test_leg_position_table(table, legs):
    for key in table.keys():
        print(key)
        for leg in legs:
            leg.set_leg_position(table["NEUTRAL"])
            time.sleep(1)
            leg.set_leg_position(table[key])
        time.sleep(1)

def test_leg_position(leg, neutral, position):
    print("going to neutral")
    leg.set_leg_position(neutral)
    print("going to leg_position")
    time.sleep(5)
    leg.set_leg_position(position)
    time.sleep(5)
    leg.set_leg_position(neutral)

def crouch_walk_test(hw):
    hw.speed = 1
    moves = [
    CROUCH_NEUTRAL,
    CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
    CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD,
    CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
    CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
    CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
    CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK,
    CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
    CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    CROUCH_NEUTRAL
    ]
    hw.do_move_set(moves)

def crouch_rotate_test(hw):
    hw.speed = 1
    moves = [
    CROUCH_NEUTRAL,
    CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
    CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT,
    CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
    CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
    CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
    CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT,
    CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
    CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
    CROUCH_NEUTRAL
    ]
    hw.do_move_set(moves)
    reverse_moves = []
    i = len(moves) - 1
    while(i >= 0):
        reverse_moves.append(moves[i])
        i = i - 1
    hw.do_move_set(reverse_moves)

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
lm = Leg(4, left_side, 3, 4, 5, 4)
lf = Leg(5, left_side, 6, 7, 8, 5)
right_legs = [rf, rm, rr]
left_legs = [lf, lm, lr]
all_legs = right_legs + left_legs


hex_walker = Hex_Walker(rf, rm, rr, lr, lm, lf)
hex_walker.do_move_set([CROUCH_NEUTRAL])
'''
for leg in all_legs:
    print("-----------------------------next leg-----------------------------")
    test_leg_position_table(NORMAL_TRI_ROTATION_TABLE, [leg])
'''

#test_leg_position(rf, CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"], CROUCH_TRI_MOVEMENT_TABLE["SIDE_LEFT"])
#test_leg_position_table(CROUCH_TRI_MOVEMENT_TABLE, [rf])
# create a test walker
#crouch_rotate_test(hex_walker)
crouch_walk_test(hex_walker)
