#!/usr/bin/env python

import sys
sys.path.append("/home/pi/Lynxmotion_Hexapod/project_files/robot_drivers/")

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import sleep
import Adafruit_PCA9685
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


global finish_command_pub
global done
done = 0

def do_command(data):
    global done
    global finish_command_pub
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
    commandlist = data.data.split()
    if commandlist[0] == "walk":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for walk"
        print("walking 4 steps forward")
        hex_walker.walk(4,0)
    if commandlist[0] == "rotate":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for rotate"
        hex_walker.rotate(1, LEFT)
    if commandlist[0] == "leg_wave":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for dance"
        hex_walker.leg_wave(LEFT, .1, 4)
    if commandlist[0] == "bounce":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for wave"
        hex_walker.bounce(.3, 4)
    done = done + 1
    finish_command_pub.publish(done)    

def node_setup():
    global finish_command_pub
    rospy.init_node('hexapod_motion_controller')
    rospy.Subscriber('motion_command', String, do_command)
    finish_command_pub = rospy.Publisher('motion_command_finished', Int32, queue_size = 1) 
    finish_command_pub.publish(0)
    rospy.spin()

if __name__ == '__main__':
    node_setup()
