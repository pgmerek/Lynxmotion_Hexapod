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
from os import getcwd


# Declare globals 
global play_started
global play_counter
global play_lines
global play_motions
global our_turn
global turing_done
# Globals for sending commands
global torso_command   # Stores the torso command that will be published
global motion_command   # Stores the walker command that will be published
global torso_done      # Increments as more motions are completed
global motion_done      # Increments as more motions are completed
global send_torso_command  # Determines whether we should send the torso command 
global send_motion_command  # Determines whether we should send the walker command 
global enable_vision    # Determines whether the vision node should be running
global vision_return    # Stores what the eyes node saw
global tts_done    # Increments as more sentences are said
global tts_command # Stores the talk command that will be published
global stt_done    # Increments as more sentences are heard
global stt_command 

# Initialize publishers
torso_command_publisher = rospy.Publisher('torso_command', String, queue_size=1)
motion_command_publisher = rospy.Publisher('torso_command', String, queue_size=1)
vision_command_publisher = rospy.Publisher('vision_command', Int32, queue_size=1)
tts_command_publisher = rospy.Publisher('talk_command', String, queue_size=1)
sst_command_publisher = rospy.Publisher('record_command', Int32, queue_size=1)
feynman_command_publisher = rospy.Publisher('feynman_command', Int32, queue_size=1)


# main has intialization values and function calls
def main():
    # Declare that variables are globals
    global torso_command
    global motion_command
    global torso_done
    global motion_done
    global send_torso_command
    global send_motion_command
    global our_turn
    global play_started
    global play_counter
    global play_motions
    global play_lines
    global turing_done
    global enable_vision
    global vision_return
    global tts_done     # tts is text to speech
    global tts_command
    global record_command
    global record_done
    global stt_done     # sst is speech to text
    global stt_command
    
    # Initialize variables
    play_started = 0
    play_counter = 0
    play_lines = []
    play_motions = []
    our_turn = 0
    turing_done = 0
    # Motion section
    torso_command = ""  # Empty strings evaluate False in Python
    motion_command = ""
    torso_done = 0
    motion_done= 0
    send_torso_command = 0
    send_motion_command = 0
    # Visual section
    enable_vision = 0
    vision_return = ["", "", ""]    # Color, size, location
    # Aural section
    tts_done = 0
    tts_command = ""
    stt_done = 0
    stt_command = ""
    record_command = ""
    record_command_done = 0

    # Get the lines, text files must be in same directory as orchestrator.py
    with open(getcwd() + '/lines.txt', 'r') as file:
        line = file.read()
        play_lines.append(line)
    # Get the motions
    with open(getcwd() + '/motions.txt', 'r') as file:
        motion = file.read()
        play_motions.append(motion)
    
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
    global play_counter
    global play_motions
    global play_lines
    global our_turn
    global enable_vision
    global vision_return
    global talk_done
    global talk_command
    global tts_done
    global tts_command
    global stt_done
    global stt_command
    global record_command
    global record_command_done

    # Intitialize node
    rospy.init_node('orchestrator', anonymous=True)
    # Refresh 10 times per second for now
    refresh_rate = rospy.Rate(10)

    # Subscribe to all input nodes 
    rospy.Subscriber('motion_command_finished', Int32, motion_command_finished_callback)
    rospy.Subscriber('torso_command_finished', Int32, torso_command_finished_callback)
    rospy.Subscriber('vision_finished', Int32, vision_finished_callback)
    rospy.Subscriber('vision_return', String, vision_return_callback)
    rospy.Subscriber('talk_command_finished', Int32, talk_command_finished_callback)
    rospy.Subscriber('record_command_finished', Int32, record_command_finished_callback)
    rospy.Subscriber('speech_command_finished', Int32, speech_command_finished_callback)
    rospy.Subscriber('turing_done', Int32, turing_done_callback)


    while not rospy.is_shutdown():
        if play_started:    # If play is started, go to an entirely different function
            execute_play()
        else:
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
            if tts_command and send_sentence:
                talk_command_publisher.publish(tts_command)
                tts_command = ""   # Reset sentence
                send_sentence = 0

            # Vision is blocking
            vision_command_publisher.publish(enable_vision)

        refresh_rate.sleep()

    # Keep python running until node is stopped
    rospy.spin()


# Function to execute the play
def execute_play():
    global play_started
    global play_counter
    global play_motions
    global play_lines
    global our_turn # True if it's our turn to speak, move, or both
    global send_motion_command
    global send_sentence

    # If it's our turn 
    if our_turn:
        # If we can talk and move
        if send_sentence and send_motion_command:
            if play_lines[play_counter] != "WAIT_FOR_TURING":   # Speak as long as the line isn't "WAIT_FOR_TURING" 
                talk_command_publisher.publish(play_lines[play_counter])    # Publish the line
                motion_command_publisher.publish(play_motions[play_counter])    # Publish the motion
                print("Sending the line \"{0}\" and the motion \"{1}\".".format(play_lines[play_counter], play_motions[play_counter]))
                play_counter += 1 # Increment counter
                send_sentence = 0   # Reset the flags for tts and motion
                send_motion_command = 0
                if play_counter > 13:   # At end of play, stop and reset for next play
                    our_turn = 0
                    play_started = 0
                    play_counter = 0
                    print("And that concludes act I of our play.")
            else:
                our_turn = 0    # Stop talking since now it's Turing's turn
                play_counter += 1 # Increment counter past the wait line. We'll still be blocked because our_turn is False
                feynman_command_publisher.publish(1)    # Publish a one to tell Turing it's his turn                
                print("Done talking and moving for now. Waiting for Turing.")
    else:
        feynman_command_publisher.publish(1)    # Publish a one to tell Turing it's his turn                
        print("Waiting for Turing")


# Callback function for Turing done
def turing_done_callback(data):
    global our_turn
    global turing_done

    if data.data == turing_done:
        pass
    else:
        our_turn = 1
        turing_done = data.data
        print("Turing said he's done with his part.")

# Callback function for speech 
def speech_command_finished_callback(data):
    global sst_done

    if data.data == sst_done:
        pass
    else:
        sst_done = data.data
        print("Done converting speech to text.")


# Callback function for record done
def record_command_finished_callback(data):
    global record_command_done

    if data.data == record_command_done:  # Do nothing unless record_done increments from the talk node
        pass
    else:
        record_command_done = data.data
        print("Record is finished.")


# Callback function for talk done
def talk_command_finished_callback(data):
    global tts_done
    global send_sentence

    if data.data == tts_done:  # Do nothing unless talk_done increments from the talk node
        pass
    else:
        send_sentence = 1
        tts_done = data.data
        print("Done converting text to speech.")


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

