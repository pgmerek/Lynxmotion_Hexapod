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

global robot_type

def main():
    host = '127.0.0.1'
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

    vikingbot0_motions = {'up'         : ['9', 'a', '6', 'w'],
                          'left_up'    : ['3', 'a', '5', 'w', '5', 'a', 'w'],
                          'left_down'  : ['3', 'a', '4', 'w', '6', 'd'],
                          'right_up'   : ['3', 's', '4', 'w', '6', 'a', 'w', 'a'],
                          'right_down' : ['5', 'a', '5', 'w', '5', 'a', 'w'],
                          'down'       : ['9', 'a', '6', 'w']}

    print ("Robot type is " + robot_type)
    if robot_type == 'hexapod':
        send_motion_command(command, hexapod_motions)
    elif robot_type == 'vikingbot0':
        send_motion_command(command, vikingbot0_motions) 
    elif robot_type == 'vikingbot1':
        send_motion_command(command, vikingbot0_motions) 


def send_motion_command(client_command, motion_command_dict):
    """
    Process the motion command from the user and send the command sequence to the
    robot motion script.
    """
    # This line will give us a list of strings
    command_sequence = motion_command_dict[client_command]
    # Some commands are sent over and over again. Keep track of this here
    command_multiplier = 0
    multiplier_present = False

    for command in command_sequence:
        # Check if we have a multiplier
        if command.isdigit():
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
    robot_type = determine_robot_model()

    # Import the functions needed for the appropriate robots
    if robot_type == "hexapod":
        sys.path.append("../../Lynxmotion_Hexapod/testing/")
        from interactive_control import *
        # Setup the globals needed for the hexpod
        setup()
    else:
        from pwm_motion import *

    main()

