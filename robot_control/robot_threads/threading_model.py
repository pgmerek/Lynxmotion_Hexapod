import threading
import time
import logging


# open cv thread
''' open cv thread will:
    -read the camera
    -do image processing
    -check if the orchestrating thread needs new info
    -send new info if it does
    -close if the quit message is sent

    the message contents will be: a list of 'seen objects' that will indicate
    {what it was, center point, size} or {what it was, top left, bottom right}

    pseudo code:

    while(!quit)
        read_camera
        do_processing
        if(orchestrating thread read last sent message)
            send new message
        sleep(amount) - should be an amount that doesn't interfere TOO much with framerate
'''

# leg thread
''' leg thread will:
    -wait for messages of what to do
    -do those actions
    -indicate that it is ready for a new action by marking the read message as finished
    -exit if the quit message is sent

    three comm channels: 1 message channel, 1 quit flag, 1 stop flag
    the message contents will be: a list of positions for the walker to go through

    pseudo code:

    while(!quit)
        -if(new message)
            for position in position list
                go to position
                sleep(.1)
                if(stop_leg_movement)
                    while(not in neutral)
                        go to current_position.closer_to_neutral_pos
                        sleep(.1)
                    break for loop
            set message to finished
        -sleep(.1) - this will stop it from eating up a ton of processor
'''

# body thread
''' body thread will:
    -wait for messages of what to do
    -do those actions
    -indicate that it is ready for a new action by marking the read message as finished
    -exit if the quit message is sent

    the message contents will be: three lists. a list of positions, a list of torso
    rotations, and a list indicating how long each of the position/rotation combos are held for
    the list of positions will correspond one-to-one with the list of rotations
    example: it was send [right arm up, right arm down, right arm up], [0, 90, 0], and [1, 4, 2]
    the robot will then rotate the torso to the 0 degree mark while putting its arm up and hold 
    for 1 second. then it will rotate to the 90 degree mark while putting its arm down and hold 
    for 4 seconds. then will rotate to the 0 degree mark while putting its arm up again and hold
    for 2 seconds

    pseudo code:

    while(!quit)
        -if(new message)
            for(length of message action list)
                do position
                do torso rotation
                sleep(amount)
                if(stop_movement_message)
                    break for loop
            tell message sender we are finished
        sleep(.1) - keep it from eating up a ton of processor
'''

# robot control thread
''' control thread will:
    -take human-understandable messages like walk [direction] [# steps], rotate [# degrees], 
    and interaction-type commands like wave, flex, point, etc... these will then be translated
    into lists of robot positions and sent to the leg/body threads
    -wait for messages from orchestrating thread, translate them for the body/leg threads, send
    those messages, and wait for those to finish. once they BOTH finish, tell the orchestrating
    thread that you are ready for a new command
    -exit if the quit message is sent

    three channels of com: 1 message, 1 quit flag, 1 stop movement flag
    message contents will be: a command from a list of commands with all fields filled out

    pseudo code:

    while(!quit)
        -if(new message)
            translate message for legs
            translate message for torso
            send leg message
            send torso message
            while(! quit && ! done)
                tmp = done
                if(leg movement NOT done)
                    tmp = not done
                if(body movement NOT done)
                    tmp = not done
                sleep(.1) - keep it from eating ton of processor power
            tell message sender we are done
        sleep(.1) - keep it from eating a ton of processor power
'''
# orchestrating program
''' orchestrating program will:
    -start all the threads with proper parameters
    -will read values from open cv
    -will determine what actions to take based on what open cv sees
    -send messages to the robot control thread


    channels of com: 1 cv message, 1 robot command message, quit flag, stop movement flag
    send out stop movement flags and stop 

    psudo code:

    -start threads
    while(!usr_quit)
        if(new cv message)
            read message
            if( read message != last message)
                set stop flag
        if(control thread finished last command)
            switch(cv message)
                enemy: send walk back message
                friend: send wave message
                food: send walk forward message
                xxxx: send rotate message
                yyyy: walk sideways
        sleep(.1) - keep it from eating a ton of processor power
    send quit message
    exit

