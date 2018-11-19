import threading
import time
import logging
from shared_thread_library import *
from robot_control_threads import *
from open_cv_thread import *

# only focus on the largest object that was seen
def parse_vision(vision_message):
    primary_object = Detected_Object("none",0)
    for obj in vision_message:
        if primary_object.size < obj.size:
            primary_object = obj
    return primary_object

def get_full_robot_action(detected_object):
    if detected_object.seen_object == "none":
        logging.debug("we don't see any objects!!!")
        return (Robot_Control_Message("stop", []), Robot_Control_Message("stop", []))

    elif detected_object.seen_object == "red":
        return (Robot_Control_Message("rotate", [-1, 1]),Robot_Control_Message("stop",[]))
    
    elif detected_object.seen_object == "green":
        if (detected_object.size < 3):
            return (Robot_Control_Message("walk", [0, 3]),Robot_Control_Message("stop",[]))
        if (detected_object.size >= 3):
            return (Robot_Control_Message("stop", []),Robot_Control_Message("stop",[]))
    
    elif detected_object.seen_object == "face":
        return (Robot_Control_Message("stop", []), Robot_Control_Message("stop",[]))

quit = threading.Event()
stop_legs = threading.Event()
stop_torso = threading.Event()


cv_to_main_msg = Message_Passer()
main_to_torso_msg = Message_Passer()
main_to_legs_msg = Message_Passer()


open_cv_thread = threading.Thread(name='open_cv_thread',
                                        target=open_cv_thread,
                                        args=(quit,cv_to_main_msg))

   
leg_control_thread = threading.Thread(name='leg_control_thread',
                                        target=leg_control_thread,
                                        args=(quit,stop_legs,main_to_legs_msg))


torso_control_thread = threading.Thread(name='torso_control_thread',
                                        target=torso_control_thread,
                                        args=(quit,stop_torso,main_to_torso_msg))


open_cv_thread.daemon = True
torso_control_thread.daemon = True
leg_control_thread.daemon = True


torso_control_thread.start()
leg_control_thread.start()
open_cv_thread.start()


last_primary_object = Detected_Object("none", 0)
primary_object = Detected_Object("none", 0)

while True:
    # check if we got a new message
    if cv_to_main_msg.message_ready_to_be_read():
        # read the message
        primary_object = parse_vision(cv_to_main_msg.read_message())
        cv_to_main_msg.set_message_read()
        logging.debug("read a new message from open cv")
        # if the main object changed, stop movement
        if not primary_object.same_object(last_primary_object):
            logging.debug("telling the robot to stop")
            stop_legs.set()
            stop_torso.set()
            while stop_legs.isSet() and stop_torso.isSet():
                pass
        
    # write messages to legs and torso 
    torso_ready = main_to_torso_msg.message_already_read()
    legs_ready = main_to_legs_msg.message_already_read()
    if (torso_ready and legs_ready):
        (leg_message, torso_message) = get_full_robot_action(primary_object)
        main_to_torso_msg.send_message(torso_message) 
        main_to_legs_msg.send_message(leg_message)
    
    ''' can be added later if wanted
    elif(torso_ready and (not legs_ready)):
        message = get_torso_only_action(primary_object)
    elif((not torso_ready) and legs_ready):
        message = get_legs_only_action(primary_object)
    '''
    last_primary_object = primary_object

    time.sleep(.1)
    #quit_now = input("Quit? 1 for yes")
    quit_now = 0
    if quit_now:
        break

quit.set()

leg_control_thread.join()
logging.debug("leg_control_thread joined")
torso_control_thread.join()
logging.debug("torso_control_thread joined")
open_cv_thread.join()
logging.debug("open_cv_thread joined")
