import threading
import time
import logging

logging.basicConfig(level=logging.DEBUG,
                    format='[%(levelname)s] (%(threadName)-10s) %(message)s',
                    )


class Message_Passer(object):
    def __init__(self):
        self.message_read = True

    def message_ready_to_be_read(self):
        return not self.message_read

    def message_already_read(self):
        return self.message_read

    def send_message(self, message):
        if(self.message_read == False):
            logging.debug("tried sending a message before the message was read")
        else:
            self.value = message
            self.message_read = False
    
    def read_message(self):
        ret = self.value
        return ret

    def set_message_read(self):
        self.message_read = True

class Detected_Object(object):
    def __init__(self, seen_object, size):
        self.seen_object = seen_object
        self.size = size

    def same_object(self, other_object):
        if self.same_object == other_object.seen_object:
            return True
        return False

    def same_size(self, other_object):
        if self.size == other_object.size:
            return True
        return False

class Body_Position_Message(object):
    def __init__(self, torso_pos, torso_rot, pause_time):
        self.torso_pos = torso_pos
        self.torso_rot = torso_rot
        self.pause_time = pause_time

def vision_message_changed(msg1, msg2):
    # if they are different lengths, vision definitely changed
    if len(msg1) != len(msg2):
        return True
    # if they are of length 0, vision did not change
    if len(msg1) == 0:
        return False
    # compare the seen objects in both messages
    for i in range(0,len(msg1)):
        if msg1[i].seen_object != msg2[i].seen_object:
            # one object was different so vision changed
            return True
    # all of them were the same so no change
    return False

''' accepted control messages are:
        rotate [1 = right -1 = left, amt]
        walk [degrees, steps]
        stop []
'''


class Robot_Control_Message(object):
    def __init__(self, action, args):
        self.action = action
        self.args = args
