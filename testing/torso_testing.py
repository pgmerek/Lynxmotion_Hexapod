import sys
sys.path.append("../robot_drivers/")

import Adafruit_PCA9685
import time
from hex_walker_driver import *

print("working")

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

def rotator_test(rotator):
    print("-----------------------------rotator test-------------------------")
    print("rotator to 0")
    rotator.set_angle(0)
    time.sleep(sleep_time)
    print("rotator to 45")
    rotator.set_angle(45)
    time.sleep(sleep_time)
    print("rotator to 65")
    rotator.set_angle(65)
    time.sleep(sleep_time)
    print("rotator to 90")
    rotator.set_angle(90)
    time.sleep(sleep_time)
    print("rotator to 115")
    rotator.set_angle(115)
    time.sleep(sleep_time)
    print("rotator to 135")
    rotator.set_angle(135)
    time.sleep(sleep_time)
    print("rotator to 180")
    rotator.set_angle(180)
    time.sleep(sleep_time)

def test_leg_position_table(table, legs):
    for key in table.keys():
        print(key)
        for leg in legs:
            leg.set_leg_position(table["NEUTRAL"])
            time.sleep(1)
            leg.set_leg_position(table[key])
        time.sleep(5)

def test_leg_positions(table, positions, leg):
    while(True):
        for pos in positions:
           # print(pos)
            leg.set_leg_position(table[pos])
            time.sleep(.4)

def test_torso_positions(positions, rotations, torso):
    while(True):
        for i in range(0,len(positions)):
            torso.set_torso_position(positions[i], rotations[i])
            time.sleep(.2)
            if i == len(positions):
                i = 0


pwm= Adafruit_PCA9685.PCA9685(address=0x40)

# create some legs
pwm.set_pwm_freq(60)
sleep_time = 5
r = Leg(6, pwm, 0, 1, 2, ARM_R)
l = Leg(7, pwm, 3, 4, 5, ARM_L)
rot = Rotator(8, pwm, 6)
#tip_motor_test(r)
#mid_motor_test(r)
#rot_motor_test(r)


#tip_motor_test(l)
#mid_motor_test(l)
#rot_motor_test(l)

#rotator_test(rot)


#test_leg_positions(TORSO_ARM_TABLE, ["HAND_SHAKE_DOWN","HAND_SHAKE_MID", "HAND_SHAKE_UP", "HAND_SHAKE_MID"], r)

#test_leg_positions(TORSO_ARM_TABLE, ["WAVE_UP", "WAVE_DOWN"], r)

torso = Robot_Torso(r, l, rot)

moves = []
moves.append(TORSO_MONKEY_RIGHT_UP)
moves.append(TORSO_MONKEY_LEFT_UP)
moves.append(TORSO_MONKEY_RIGHT_UP)
moves.append(TORSO_MONKEY_LEFT_UP)
moves.append(TORSO_MONKEY_RIGHT_UP)
moves.append(TORSO_MONKEY_LEFT_UP)
moves.append(TORSO_MONKEY_RIGHT_UP)
moves.append(TORSO_MONKEY_LEFT_UP)
rotations = [45, 45, 45, 45, 135, 135, 135,135]

test_torso_positions(moves, rotations, torso)
