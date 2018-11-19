from shared_thread_library import *

testing = 1

def open_cv_thread(quit, cv_to_main_msg):
    object_iter = 0
    seen_objects = [
            Detected_Object("red", 1),
            Detected_Object("red", 2),
            Detected_Object("red", 3),
            Detected_Object("green", 1),
            Detected_Object("green", 2),
            Detected_Object("green", 3),
            Detected_Object("face", 1),
            Detected_Object("face", 2),
            Detected_Object("face", 3)
        ]

    while not quit.isSet():
        vision_message = []
        if(testing == 1):
            if(object_iter >= len(seen_objects)):
                obj =  Detected_Object("none", 0)
            else:
                obj = seen_objects[object_iter]
            object_iter = object_iter + 1
            vision_message.append(obj)
            time.sleep(2) # sleep to simulate open cv slowness
        else:
            logging.debug("not testing i guesss???")
            # actual object finding code goes here
        
        
        if(cv_to_main_msg.message_already_read()):
            logging.debug("open cv thread just sent a message")
            cv_to_main_msg.send_message(vision_message)
        time.sleep(.1)
