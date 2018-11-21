import sys
sys.path.append("..")

from hex_walker_driver import *
from shared_thread_library import *

def execute_leg_action(legs_msg):
    # here we have logic for doing actinos based on the message
    if(legs_msg.action == "stop"):
        logging.debug("legs are stopping")
        time.sleep(2)
    
    elif(legs_msg.action == "walk"):
        direction = legs_msg.args[0]
        steps = legs_msg.args[1]
        logging.debug("walking "+str(steps)+" steps in direction "+str(direction))
        time.sleep(2)

    elif(legs_msg.action == "rotate"):
        direction = legs_msg.args[0]
        steps = legs_msg.args[1]
        logging.debug("rotating "+str(direction)+" for "+str(steps)+" num steps")
        time.sleep(2)

def leg_control_thread(quit, stop_legs, main_to_legs_msg):
    # instantiate a hex walker
    logging.debug("leg_control_thread_running")
    while not quit.isSet():
        if main_to_legs_msg.message_ready_to_be_read():
            execute_leg_action(main_to_legs_msg.read_message())
            main_to_legs_msg.set_message_read()
        if stop_legs.isSet():
            logging.debug("stopping legs")
            stop_legs.clear()
        time.sleep(.1)
        
def execute_torso_action(torso_msg):
    # here we have logic for doing actions based on the message
    if(torso_msg.action == "stop"):
        logging.debug("torso is stopping")
            
def torso_control_thread(quit, stop_torso, main_to_torso_msg):
    logging.debug("torso_control_thread_running")
    while not quit.isSet():
        if main_to_torso_msg.message_ready_to_be_read():
            execute_torso_action(main_to_torso_msg.read_message())
            main_to_torso_msg.set_message_read()
        if stop_torso.isSet():
            logging.debug("stopping torso")
            stop_torso.clear()
        time.sleep(.1)

