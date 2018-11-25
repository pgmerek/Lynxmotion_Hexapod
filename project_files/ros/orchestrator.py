#!/usr/bin/env python
"""
Orchestrator
ECE 578
11/24/2018

Author: Patrick Gmerek
"""

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


# Declare globals 
global play_started
# Globals for sending commands
global motion_command   # Stores the motion command that will be published
global motion_done      # Increments as more motions are completed
global send_motion_command  # Determines whether we should send the motion command 

global enable_vision    # Determines whether the vision node should be running

# Initialize publishers
motion_command_publisher = rospy.Publisher('motion_command', String, queue_size=1)
vision_command_publisher = rospy.Publisher('vision_command', Int32, queue_size=1)


# main has intialization values and function calls
def main():
    # Declare that variables are globals
    global play_started
    global motion_done
    global send_motion_command
    global motion_command
    global enable_vision
    
    # Initialize variables
    play_started = 0
    motion_done = 0
    send_motion_command = 0
    motion_command = "Do_Nothing"
    enable_vision = 0
   
    # Call line_follower but capture exception if thrown
    try:
        print("Starting orchestrator")
        orchestrator()
    except rospy.ROSInterruptException as e:
        print(e)


# Collection of publisher and subscribers
def orchestrator():
    global play_started
    global motion_done
    global motion_command
    global send_motion_command
    global enable_vision
    
    new_motion_command = "Do_Nothing"   # This variable holds the next, requested motion. Actual motion relies on the global motion_command

    # Intitialize node
    rospy.init_node('orchestrator', anonymous=True)
    # Refresh 10 times per second for now
    refresh_rate = rospy.Rate(10)

    # Subscribe to all input nodes 
    rospy.Subscriber('motion_command_finished', Int32, motion_command_finished_callback)

    # Publish to /motion_command every 5 seconds
    while not rospy.is_shutdown():
        # Publish a "Do_Nothing" command or the new command if ready
        motion_command_publisher.publish(motion_command_handler(new_motion_command))
        
        vision_command_publisher.publish(enable_vision)
        refresh_rate.sleep()

    # Keep python running until node is stopped
    rospy.spin()


# Publisher function for motion command 
def motion_command_handler(new_motion_command):
    global send_motion_command
    motion_command = "Do_Nothing"

    if send_motion_command:     # If we are cleared to send a command, set the return to the new command and reset send_motion_command
        motion_command = new_motion_command 
        send_motion_command = 0

    return motion_command


# Callback function for motion command finished 
def motion_command_finished_callback(data):
    global motion_done
    global send_motion_command

    if data.data == motion_done:  # Do nothing unless the motion_command_done increments
        pass
    else:
        send_motion_command = 1
        motion_done = data.data
        

if __name__ == "__main__":
    main()

