"""
Script to receive commands to the Raspberry Pi using a socket server

Author: Charles Stoll, Patrick Gmerek
ECE 579
"""
import sys

import socket
import os
import re
import subprocess
import time
from Adafruit_BNO055 import BNO055

global robot_type
global starting_orientation

def main():
    host = ''
    port = 65432

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(5)

    # In degrees, with the 0 degress pointing straight up
    client_input = '' 
    print('Waiting for connection...')

    while True:
        connection, client_address = s.accept()
        print('Connection from ', client_address)
        client_input = connection.recv(20).decode('ascii')
        print('Received {}'.format(client_input))
        if client_input:
            move_robot(client_input)

            # Send acknowledge signal back to client
            data = b'ack'
            connection.sendall(data)
            client_input = ''
        else:
            print('no data from', client_address)


def get_orientation():
    global starting_orientation

    # Query the real heading
    heading, roll, pitch = bno.read_euler()

    # Return the difference between the current heading and the starting orientation
    return heading - starting_orientation

def correct_for_drift():
    global starting_orientation

    # Query the real heading
    heading, roll, pitch = bno.read_euler()

    print("In correct_for_drift(), we are comparing the current heading {0:0.2F} against the starting orientation {0:0.2F}".
            format(heading, starting_orientation))
    
    # Arbitrary threshold is set up right now. Need to test the ripple dance cycle to find out
    # how precisely and accurately we can rotate the hexapod

def move_robot(command):
    """
    Looks in the current working directory for a text file with the type of robot
    that this script is running on.
    """
    global robot_type
    hexapod_motions = {'up'         : ['9', 'wd', 'w'],
                       'left_up'    : ['9',  'w', 'sd'],
                       'left_down'  : ['7', 'wa', 'w', 'd'],
                       'right_up'   : [''],
                       'right_down' : [''],
                       'down'       : ['10', 'sa', '2', 'a', 'w']}

    vikingbot1_motions = {'up'          : ['1.35', 'a', '1.5', 'w'],
                          'left_up'     : ['0.68', 'a', '1.3', 'w', '0.74', 'a', '0.4', 's'],
                          'left_down'   : ['0.8', 'd', '1.1', 'w', '0.8', 'd', '0.6', 's'],
                          'right_up'    : ['0.7', 'd', '1.1', 'w', '0.7', 'd', '0.2', 's'],
                          'right_down'  : ['0.7', 'a', '1.1', 'w', '0.7', 'a', '0.5', 's'], 
                          'down'        : ['1.35', 'a', '1.35', 'w']}
    vikingbot0_motions = {'up'         : ['1.05', 'a', '2.2', 'w'],
                          'left_up'    : ['0.6', 'a', '1.5', 'w', '0.6', 'a', '0.9', 's'],
                          'left_down'  : ['0.6', 'd', '1.5', 'w', '0.6', 'd', '0.9', 's'],
                          'right_up'   : ['0.6', 'd', '1.5', 'w', '0.6', 'd', '0.9', 's'],
                          'right_down' : ['0.6', 'a', '1.5', 'w', '0.6', 'a', '0.9', 's'],
                          'down'       : ['1.05', 'a', '2.2', 'w']}

    print ("Robot type is " + robot_type)
    if robot_type == 'hexapod':
        correct_for_drift()
        send_motion_command(command, hexapod_motions)
    elif robot_type == 'vikingbot0':
        send_motion_command(command, vikingbot0_motions) 
    elif robot_type == 'vikingbot1':
        send_motion_command(command, vikingbot1_motions) 

def send_motion_command(client_command, motion_command_dict):
    """
    Process the motion command from the user and send the command sequence to the
    robot motion script.
    """
    global robot_type
    global starting_orientation
    # This line will give us a list of strings
    command_sequence = motion_command_dict[client_command]
    # Some commands are sent over and over again. Keep track of this here
    command_multiplier = 0
    multiplier_present = False
    command_duration = 0

    for command in command_sequence:
        if robot_type is 'hexapod':
            # Check if we have a multiplier
            if command.isdigit():
                if robot_type is 'hexapod':
                    command_multiplier = int(command)
                    multiplier_present = True
            else:
                if multiplier_present:
                    # Send the command X times
                    for x in range(0, command_multiplier + 1):
                        print("Sending {}".format(command))
                        command_arbiter(command)
                    multiplier_present = False
                else:
                    print("Sending {}".format(command))
                    command_arbiter(command)

        else:
            try:
                command_duration = float(command)
            except ValueError:
                command_arbiter(command, command_duration)

def determine_robot_model():
    # Check what kind of robot this script is running on
    robot_type = ''
    with open ('robot_type.txt', 'r') as f:
        robot_type = (f.readline()).rstrip('\n')

    if robot_type == 'hexapod':
        pass
    elif robot_type == 'vikingbot0':
        pass
    elif robot_type == 'vikingbot1':
        pass
    else:
        print("No valid robot type specified. Read \"{}\", which is not a valid robot.".format(robot_type))
        sys.exit(1)

    return robot_type


if __name__ == "__main__":
    # Store the model of the robot this script is running on
    global robot_type
    global starting_orientation
    robot_type = determine_robot_model()

    bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
    heading, roll, pitch = bno.read_euler()
    starting_orientation = heading
    print("Starting orientation (virtual North) is {0:0.2F}".format(starting_orientation)) 

    if not bno.begin():
        raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

    # Import the functions needed for the appropriate robots
    if robot_type == "hexapod":
        sys.path.append("../../Lynxmotion_Hexapod/testing")
        from interactive_control import *
        # Setup the globals needed for the hexpod
        setup()
    else:
        from pwm_motion import *
        setup()

    try:
        main()
    except KeyboardInterrupt:
        print("Received keyboard interrupt!")
        if robot_type == 'vikingbot0':
            GPIO.cleanup()
        sys.exit()

