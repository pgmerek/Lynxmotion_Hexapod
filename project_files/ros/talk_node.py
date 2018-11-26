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

global talk_done
talk_done = 0

global talk_command

pub_talk_done = rospy.Publisher("talk_done", Int32, queue_size=1)

def talk_callback(data):
	
	string text

	polly = boto3.client('polly')

	text = data.data
	spoken_text = polly.synthesize_speech(text, OutputFormat='mp3', VoiceId = 'Matthew')

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


def robot_main():
    pass


def talk_node_setup():

	global talk_done
	
	# initialize the node
	rospy.init_node("talk_node_setup", anonymous = True)

	# set the publishing rate
	rate = rospy.Rate(10) #10Hz

	# create a subscriber
	rospy.Subscriber("talk_command", String, talk_callback)

	# publish command of the text and int
	while not rospy.is_shutdown():
            robot_main()
            talk_done = talk_done + 1
            pub_talk_command.publish(talk_done)
            rate.sleep()
	rospy.spin()
	
if __name__ == '__main__':
	try:
		talk_node_setup()
	except rospy.ROSInterruptException():
		pass

