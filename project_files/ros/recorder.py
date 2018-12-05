#!/usr/bin/env python

import sys
sys.path.append("/home/pi/Lynxmotion_Hexapod/project_files/robot_drivers/")

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import sleep
import Adafruit_PCA9685
from hex_walker_driver import *
import os



def record(file_name, seconds):
    bash_command = "arecord -D plughw:1,0 -c1 -r44100 -d " + str(seconds) + " " + file_name
    os.system(bash_command)

global path
global finish_record_pub
global file_path_pub
global record_done
record_done = 0

def do_record(data):
    global record_done
    global finish_hex_pub
    global file_path_pub
     
    path = "/home/pi/catkin_ws/src/lynxmotion_package/wavs/"
    file_name = path + "microphone" + str(record_done) + ".wav"

    rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
    seconds = data.data
    
    record(file_name, seconds)

    record_done = record_done + 1
    file_path_pub.publish(file_name)
    finish_record_pub.publish(record_done)    

def node_setup():
    global finish_record_pub
    global finish_path_pub 
    global file_path_pub
    rospy.init_node('recorder')
    rospy.Subscriber('record_command', Int32, do_record)
    finish_record_pub = rospy.Publisher('record_command_finished', Int32, queue_size = 1)
    file_path_pub = rospy.Publisher('file_recorded', String, queue_size = 1)
    finish_record_pub.publish(0)
    rospy.spin()

if __name__ == '__main__':
    node_setup()
