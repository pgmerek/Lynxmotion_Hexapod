from leg_data import *

class Robot_Position(object):
    def __init__(self, rf_pos, rm_pos, rr_pos, lr_pos, lm_pos, lf_pos):
        self.rf_pos = rf_pos
        self.rm_pos = rm_pos
        self.rr_pos = rr_pos
        self.lr_pos = lr_pos
        self.lm_pos = lm_pos
        self.lf_pos = lf_pos

    def __str__(self):
        start_str = "--------------------------robot position is------------------\n"
        rf_str = "rf: " + str(rf_pos) + "\n"
        rm_str = "rm: " + str(rm_pos) + "\n"
        rr_str = "rr: " + str(rr_pos) + "\n"
        lr_str = "lr: " + str(lr_pos) + "\n"
        lm_str = "lm: " + str(lm_pos) + "\n"
        lf_str = "lf: " + str(lf_pos) + "\n"
        return start_str + rf_str + rm_str + rr_str + lr_str + lm_str + lf_str

# NOTE: I have left in repeated steps and simply commented them out. It helps for continuiuty and error checking since you can see the entire process

# all possible robot positions
# possible robot positions during a tripod "walk" cycle
NORMAL_NEUTRAL = 1
NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL = 2
NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD = 3
NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD = 4
NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD = 5
NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL = 6
NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK = 7
NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK = 8
NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK = 9
#NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possiblt robot positions during a tripod "rotate" cycle
#NORMAL_TRI_NEUTRAL
#NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT = 10
NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT = 11
NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT = 12
#NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT = 13
NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT = 14
NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT = 15
#NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL


# these are all defines as robot_position(rf, rm, rr, lr, lm, lf)
ROBOT_POSITIONS = {
        NORMAL_NEUTRAL : Robot_Position(                            NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"]),
        
        NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL : Robot_Position(  NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["UP_NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["UP_NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                                                                    NORMAL_TRI_ROTATION_TABLE["UP_NEUTRAL"])
        }
































