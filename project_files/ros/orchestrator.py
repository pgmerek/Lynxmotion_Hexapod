#!/usr/bin/env python
"""
Orchestrator
ECE 578
11/24/2018

Author: Patrick Gmerek
"""

import time
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
# Torso
global torso_command   # Stores the torso command that will be published
global torso_done      # Increments as more motions are completed
global send_torso_command  # Determines whether we should send the torso command 
# Legs
global motion_command   # Stores the walker command that will be published
global motion_done      # Increments as more motions are completed
global send_motion_command  # Determines whether we should send the walker command 
# Text to speech
global talk_command # Stores a string of what we want to convert to speech
global talk_done    # Increments as more sentences are said
global talk     # Lets us know we are allowed to speak
# Speech to text
global dialog_finished  # Increments as more sentences are processed 
global respond  # Lets us know that we have a valid sentence
global dialog_response   # Lets use know that we should get the speech now that it's processed
# Record
global file_path    # Return the file path of the recording file
global record_done  # Increments as more commands are recorded
global listen   # Lets us know that we should be listening
global recording_ready    # Lets us know that recording is ready
global start_time   # Stores when we started the orchestrator
global rec_len  # The number of seconds that we're recording

# Initialize variables starting with play variables
play_started = 1
play_counter = 0
play_lines = []
play_motions = []
our_turn = 0
turing_done = -1
# Torso section
torso_command = ""
torso_done = 0
send_torso_command = 1
# Legs section
motion_command = ""
motion_done= 0
send_motion_command = 1
# Text to Speech section
talk_command = ""
talk_done = 0
talk = 0
# Speech to text section
dialog_response = ""
dialog_finished = 0
respond = 0
# Record section
file_path = ""
record_done = 0
listen = 0
recording_ready = 0
start_time = int(time.time())
rec_len = 5

# Initialize publishers
torso_command_publisher = rospy.Publisher('torso_command', String, queue_size=1)
motion_command_publisher = rospy.Publisher('motion_command', String, queue_size=1)
talk_command_publisher = rospy.Publisher('talk_command', String, queue_size=1)
record_command_publisher = rospy.Publisher('record_command', Int32, queue_size=1)
feynman_done_publisher = rospy.Publisher('feynman_done', Int32, queue_size=1)
dialog_command_publisher = rospy.Publisher('dialog_command', String, queue_size=1)


# Get the lines, text files must be in same directory as orchestrator.py
with open(getcwd() + '/lines.txt', 'r') as file:
    line = file.readline()
    while line:
        play_lines.append(line)
        line = file.readline()
# Get the motions
with open(getcwd() + '/motions.txt', 'r') as file:
    motion = file.readline()
    while motion:
        play_motions.append(motion)
        motion = file.readline()


# Collection of publisher and subscribers
def orchestrator():
    global play_started
    global send_torso_command
    global torso_command
    global send_motion_command
    global motion_command
    global talk 
    global talk_command
    global respond  # Respond to small talk 
    global speech_command
    global recording_ready
    # Needed since we only want to publish once every 5 seconds
    curr_time = 0
    prev_time = 0  
    
    # Intitialize node
    rospy.init_node('orchestrator', anonymous=True)
    # Refresh 10 times per second for now
    refresh_rate = rospy.Rate(10)

    # Subscribe to all input nodes 
    rospy.Subscriber('motion_command_finished', Int32, motion_command_finished_callback)
    rospy.Subscriber('torso_command_finished', Int32, torso_command_finished_callback)
    rospy.Subscriber('talk_done', Int32, talk_done_callback)
    rospy.Subscriber('record_command_finished', Int32, record_command_finished_callback)
    rospy.Subscriber('file_recorded', String, file_recorded_callback)
    rospy.Subscriber('turing_done', Int32, turing_done_callback)
    rospy.Subscriber('dialog_finished', Int32, dialog_finished_callback)
    rospy.Subscriber('dialog_response', String, dialog_response_callback)


    while not rospy.is_shutdown():
        if play_started:    # If play is started, go to an entirely different function
            execute_play()
        else:
            curr_time = int(time.time()) - start_time 
            if curr_time != prev_time and curr_time % rec_len == 0:   # If 5 seconds have elapsed
                prev_time = curr_time
                record_command_publisher.publish(rec_len)
                print("tick")
            # Get file path from Charles, send it to Emma
            if recording_ready:
                dialog_command_publisher.publish(file_path)
                recording_ready = 0
            # If we are allowed to send a torso command, send it
            if send_torso_command:  
                torso_command_publisher.publish(torso_command)
                send_torso_command = 0
            # If we are allowed to send a motion command, send it
            if send_motion_command:
                motion_command_publisher.publish(motion_command)
                send_motion_command = 0
            # If we are allowed to say something, send it
            if talk:
                talk_command_publisher.publish(talk_command)
                talk = 0
            # If there is something we heard that should be acted on, act on it
            if respond:
                small_talk(dialog_response)
                respond = 0

        refresh_rate.sleep()

    # Keep python running until node is stopped
    rospy.spin()


# Function for small talk
def small_talk(user_sentence):
    global talk_command
    global talk

    if user_sentence == "Hello":
        talk_command = "Hello."
    elif user_sentence == "Hi":
        talk_command = "Howdy."
    elif user_sentence == "How are you?":
        talk_command = "Excellent."
    elif user_sentence == "Is it raining outside?":
        talk_command = "Why don't you go outside and check?"
    elif user_sentence == "Goodbye":
        talk_command = "See you later alligator."
    elif user_sentence == "Bye":
        talk_command = "Goodbye."
    elif user_sentence == "What are you?":
        talk_command = "I am nothing short of an abomination."
    elif user_sentence == "What's up?":
        talk_command = "Not much. Just having a smoke."
    elif user_sentence == "You seem sad":
        talk_command = "I suppose I could use a leg up."
    else:
        talk_command = "I don't understand what you said."

    talk = 1


# Function to execute the play
def execute_play():
    global play_started
    global play_counter
    global play_motions
    global play_lines
    global our_turn # True if it's our turn to speak, move, or both
    global send_motion_command
    global talk

    talk = 1
    # If it's our turn 
    if our_turn:
        current_line = play_lines[play_counter].rstrip('\n')
        current_motion = play_motions[play_counter].rstrip('\n')
        # If we can talk and move
        if talk and send_motion_command and send_torso_command:
            if current_line != "WAIT_FOR_TURING":   # Speak as long as the line isn't "WAIT_FOR_TURING" 
                talk_command_publisher.publish(current_line)    # Publish the line
                motion_command_publisher.publish(current_motion)    # Publish the motion
                torso_command_publisher.publish(current_motion)    # Publish the motion
                print("Sending the line \"{0}\" and the motion \"{1}\".".format(current_line, current_motion))
                play_counter += 1 # Increment counter
                talk = 0   # Reset the flags for tts and motion
                send_motion_command = 0
                if play_counter > 13:   # At end of play, stop and reset for next play
                    our_turn = 0
                    play_started = 0
                    play_counter = 0
                    print("And that concludes act I of our play.")
            else:
                our_turn = 0    # Stop talking since now it's Turing's turn
                play_counter += 1 # Increment counter past the wait line. We'll still be blocked because our_turn is False
                feynman_done_publisher.publish(play_counter)    # Publish a one to tell Turing it's his turn                
                print("Done talking and moving for now. Waiting for Turing.")
    else:
       # feynman_done_publisher.publish(play_counter)    # Publish a new integer to tell Turing it's his turn                
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


# Callback function for dialog done
def dialog_response_callback(data):
    global dialog_response

    dialog_response = data.data


# Callback function for dialog done
def dialog_finished_callback(data):
    global dialog_finished
    global respond

    if data.data == dialog_finished:  # Do nothing unless dialog_finished increments from the dialog node
        pass
    else:
        dialog_finished = data.data
        respond = 1;


# Callback function for file_recorded
def file_recorded_callback(data):
    global record_return

    record_return = data.data


# Callback function for record done
def record_command_finished_callback(data):
    global record_done
    global recording_ready

    if data.data == record_done:  # Do nothing unless record_done increments from the talk node
        pass
    else:
        record_done = data.data
        recording_ready = 1
        print("Done recording.")


# Callback function for talk done
def talk_done_callback(data):
    global talk_done
    global talk

    if data.data == talk_done:  # Do nothing unless talk_done increments from the talk node
        pass
    else:
        talk_done = data.data
        talk = 1
        print("Done converting text to speech.")


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
    # Call orchestrator but capture exception if thrown
    try:
        print("Starting orchestrator")
        orchestrator()
    except rospy.ROSInterruptException as e:
        print(e)

