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

global finish_hex_pub
global finish_torso_pub
global torso_done
global hex_done
torso_done = 0
hex_done = 0

def do_hex_walker_command(data):
    global hex_done
    global finish_hex_pub
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
    commandlist = data.data.split()
    
    
    if commandlist[0] == "walk":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for walk"
        num_steps = int(commandlist[1])
        direction = int(commandlist[2])
        print("walking " + str(num_steps) + " steps forward in direction " + str(direction))
        hex_walker.walk(num_steps, direction)
    
    
    if commandlist[0] == "rotate":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for rotate"
        num_steps = int(commandlist[1])
        if(commandlist[2] == "left"):
            direction = LEFT
        else:
            direction = RIGHT
        print("rotating " + str(num_steps) + commandlist[2])
        hex_walker.rotate(num_steps, direction)
    
    
    if commandlist[0] == "leg_wave":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for leg_wave"
        num_times = int(commandlist[1])
        if(commandlist[2] == "left"):
            direction = LEFT
        else:
            direction = RIGHT
        print("leg waving " + str(num_times) + " in the direction " + commandlist[2])
        hex_walker.leg_wave(direction, .1, num_times)
    
    
    if commandlist[0] == "bounce":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for bounce"
        num_times = int(commandlist[1])
        wait_time = float(commandlist[2])
        hex_walker.bounce(wait_time, num_times)


    hex_done = hex_done + 1
    finish_hex_pub.publish(hex_done)    

def do_torso_command(data):
    global torso_done
    global finish_torso_pub
    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
    commandlist = data.data.split()
    
    
    if commandlist[0] == "wave":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for wave"
        num_times = int(commandlist[1])
        direction = int(commandlist[2])
        print("waving " + str(num_times) + " times in direction " + str(direction))
        torso.wave(direction, num_times)


    if commandlist[0] == "king_kong":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for king_kong"
        num_times = int(commandlist[1])
        direction = int(commandlist[2])
        print("king_kong-ing " + str(num_times) + " times in direction " + str(direction))
        torso.king_kong(direction, num_times)
    
        
    if commandlist[0] == "monkey":
        if len(commandlist) != 2:
            assert "incorrect number of arguments for monkey"
        num_times = int(commandlist[1])
        print("monkeying " + str(num_times) + " times")
        torso.monkey(num_times)
    
        
    if commandlist[0] == "handshake":
        if len(commandlist) != 3:
            assert "incorrect number of arguments for handshake"
        num_times = int(commandlist[1])
        direction = int(commandlist[2])
        torso.hand_shake(direction, num_times) 
    
        
    torso_done = torso_done + 1
    finish_torso_pub.publish(torso_done)    


def node_setup():
    global finish_hex_pub
    global finish_torso_pub
    rospy.init_node('hexapod_motion_controller')
    rospy.Subscriber('motion_command', String, do_hex_walker_command)
    rospy.Subscriber('torso_command', String, do_torso_command)
    finish_hex_pub = rospy.Publisher('motion_command_finished', Int32, queue_size = 1) 
    finish_torso_pub = rospy.Publisher('torso_command_finished', Int32, queue_size = 1)
    finish_hex_pub.publish(0)
    finish_torso_pub.publish(0)
    rospy.spin()

if __name__ == '__main__':
    node_setup()
