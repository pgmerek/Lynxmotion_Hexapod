#!/usr/bin/env python

# This is the node file for the text to speech

import boto3
from pygame import mixer
import os

import rospy
from std_msgs.msg import Int32
from std_msgs.msg import String


#Here are the installation lines
# pip install awscli
# aws configure
# pip install boto3
# pip install pygame

global talk_finished
talk_finished = 0

#This is the ROS topic that the talk node is subscribed to
global talk_command

# Previous variable to keep track of what was said
global previous
previous = ""

def talk_callback(data):

	global pub_talk_done	
	global talk_finished
	global previous

	polly = boto3.client('polly')

        # If the same string is being published, skip
        if data.data == previous:
                pass
        else:
                previous = data.data # update previous variable
                spoken_text = polly.synthesize_speech(Text = data.data, OutputFormat='mp3', VoiceId = 'Matthew')

                with open('output.mp3', 'wb') as f:
                    f.write(spoken_text['AudioStream'].read())
                    f.close()

                mixer.init()
                mixer.music.load('output.mp3')
                mixer.music.play()

                while mixer.music.get_busy() == True:
                    pass

                mixer.quit()
                os.remove('output.mp3')

                talk_finished = talk_finished + 1 # Keeps track of how many times polly has talked
                pub_talk_done.publish(talk_finished)
                print("This is the end of the callback")
	

def talk_node_setup():
	
	global pub_talk_done
	
	# initialize the node
	rospy.init_node("talk_node_setup")


	# create a subscriber
	rospy.Subscriber("talk_command", String, talk_callback)

	pub_talk_done = rospy.Publisher("talk_done", Int32, queue_size=1)
	pub_talk_done.publish(0)

	rospy.spin()
	
if __name__ == '__main__':
	try:
		talk_node_setup()
	except rospy.ROSInterruptException():
		pass

