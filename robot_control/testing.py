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
            leg.set_leg_position(table[key])
        time.sleep(sleep_time)

#init the pwm stuffs and run selected tests
pwm40 = Adafruit_PCA9685.PCA9685(address=0x40)
pwm40.set_pwm_freq(60)
sleep_time = 1
leg_right_mid= Leg(1, pwm40, 0, 1, 2, RIGHT)
leg_right_front = Leg(0, pwm40, 3, 4, 5, RIGHT)
leg_left_front = Leg(5, pwm40, 6, 7, 8, LEFT)
leg_left_mid = Leg(4, pwm40, 9, 10, 11, LEFT)
legs = [leg_right_mid, leg_right_front, leg_left_front, leg_left_mid]
test_leg_position_table(NORMAL_TRI_ROTATION_TABLE, legs)

