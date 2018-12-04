#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from time import sleep

global feynman_done
global turing_done 
feynman_done = 0
turing_done = 0

turing_done_publisher = rospy.Publisher('turing_done', Int32, queue_size=1)

def node_setup():
    print("Starting up dummy Turing node.")
    rospy.init_node('turing')
    rospy.Subscriber('feynman_done', Int32, feynman_done_callback)
    turing_done_publisher.publish(turing_done)
    rospy.spin()


def feynman_done_callback(data):
    global feynman_done
    global turing_done

    if feynman_done == data.data:
        pass
    else:
        feynman_done = data.data
        sleep(1)
        print("Telling Feynman that Turing is done.")
        turing_done += 1
        turing_done_publisher.publish(turing_done)

    
if __name__ == '__main__':
    node_setup()

