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
global torso_command   # Stores the torso command that will be published
global motion_command   # Stores the walker command that will be published
global torso_done      # Increments as more motions are completed
global motion_done      # Increments as more motions are completed
global send_torso_command  # Determines whether we should send the torso command 
global send_motion_command  # Determines whether we should send the walker command 
global enable_vision    # Determines whether the vision node should be running
global vision_return    # Stores what the eyes node saw
global talk_done    # Increments as more sentences are said
global talk_command # Stores the talk command that will be published

# Initialize publishers
torso_command_publisher = rospy.Publisher('torso_command', String, queue_size=1)
motion_command_publisher = rospy.Publisher('torso_command', String, queue_size=1)
vision_command_publisher = rospy.Publisher('vision_command', Int32, queue_size=1)
talk_command_publisher = rospy.Publisher('talk_command', String, queue_size=1)


# main has intialization values and function calls
def main():
    # Declare that variables are globals
    global torso_command
    global motion_command
    global torso_done
    global motion_done
    global send_torso_command
    global send_motion_command
    global play_started
    global enable_vision
    global vision_return
    global talk_done
    global talk_command
    
    # Initialize variables
    torso_command = ""  # Empty strings evaluate False in Python
    motion_command = ""
    torso_done = 0
    motion_done= 0
    send_torso_command = 0
    send_motion_command = 0
    play_started = 0
    enable_vision = 0
    vision_return = ["", "", ""]    # Color, size, location
    talk_done = 0
    talk_command = ""
   
    # Call orchestrator but capture exception if thrown
    try:
        print("Starting orchestrator")
        orchestrator()
    except rospy.ROSInterruptException as e:
        print(e)


# Collection of publisher and subscribers
def orchestrator():
    global torso_command
    global motion_command
    global torso_done
    global motion_done
    global send_torso_command
    global send_motion_command
    global play_started
    global enable_vision
    global vision_return
    global talk_done
    global talk_command

    # Intitialize node
    rospy.init_node('orchestrator', anonymous=True)
    # Refresh 10 times per second for now
    refresh_rate = rospy.Rate(10)

    # Subscribe to all input nodes 
    rospy.Subscriber('motion_command_finished', Int32, motion_command_finished_callback)
    rospy.Subscriber('torso_command_finished', Int32, torso_command_finished_callback)
    rospy.Subscriber('vision_finished', Int32, vision_finished_callback)
    rospy.Subscriber('vision_return', String, vision_return_callback)
    rospy.Subscriber('talk_done', Int32, talk_done_callback)

    while not rospy.is_shutdown():
        # If we have a motion command and we are cleared to send it, send it
        if motion_command and send_motion_command:
            motion_command_publisher.publish(motion_command)
            motion_command = "" # Reset flag and motion command
            send_motion_command = 0

        # If we have a torso command and we are cleared to send it, send it
        if torso_command and send_torso_command:  
            torso_command_publisher.publish(torso_command)
            torso_command = "" # Reset flag and torso command
            send_torso_command = 0

        # If we have a sentence to say and we are allowed to say it, send it
        if talk_command and send_sentence:
            talk_command_publisher.publish(talk_command)
            talk_command = ""   # Reset sentence
            send_sentence = 0

        # Vision is blocking
        vision_command_publisher.publish(enable_vision)

        refresh_rate.sleep()

    # Keep python running until node is stopped
    rospy.spin()


# Callback function for talk done
def talk_done_callback(data):
    global talk_done
    global send_sentence

    if data.data == talk_done:  # Do nothing unless talk_done increments from the talk node
        pass
    else:
        send_sentence = 1
        talk_done = data.data
        print("Talk is finished.")


# Callback function for vision return
def vision_return_callback(data):
    global vision_return
    temp_string = data.data.split()

    if len(temp_string) == 3:   # If we received a valid return from eyes
        vision_return[0] = temp_string[0]
        vision_return[1] = temp_string[1]
        vision_return[2] = temp_string[2]
        print("Eyes saw {0} colored object of {1} size and {2} location."
                .format(vision_return[0], vision_return[1], vision_return[2]))


# Callback function for vision finished 
def vision_finished_callback(data):
    global vision_done
    global enable_vision

    if data.data == vision_done:  # Do nothing unless the vision_done increments
        pass
    else:
        enable_vision = 0
        vision_done = data.data
        print("Vision is finished.")


# Callback function for torso command finished 
def torso_command_finished_callback(data):
    global torso_done
    global send_torso_command

    if data.data == torso_done:  # Do nothing unless the torso_done increments
        pass
    else:
        send_torso_command = 1
        torso_done = data.data
        print("Torso command is finished.")


# Callback function for motion command finished 
def motion_command_finished_callback(data):
    global motion_done
    global send_motion_command

    if data.data == motion_done:  # Do nothing unless the motion_done increments
        pass
    else:
        send_motion_command = 1
        motion_done = data.data
        print("Motion command is finished.")
        

if __name__ == "__main__":
    main()

