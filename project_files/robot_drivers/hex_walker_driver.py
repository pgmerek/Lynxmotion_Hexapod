from __future__ import division
import time
from hex_walker_data import *
from leg_data import *
from torso_data import *

# uncomment if working with the actual robot
#import Adafruit_PCA9685

#Extraneous
HW_MOVE_DEBUG = 1 #toggle 0/1 to turn debug prints on/off

# all constants related to the hex_walker legs
c_0_TIP_MOTOR_OUT = 185
c_0_TIP_MOTOR_IN = 480
c_0_MID_MOTOR_UP = 612
c_0_MID_MOTOR_DOWN = 264
c_0_ROT_MOTOR_RIGHT = 149
c_0_ROT_MOTOR_LEFT = 603

c_1_TIP_MOTOR_OUT = 153
c_1_TIP_MOTOR_IN = 459
c_1_MID_MOTOR_UP = 604
c_1_MID_MOTOR_DOWN = 259
c_1_ROT_MOTOR_RIGHT = 154
c_1_ROT_MOTOR_LEFT = 603

c_2_TIP_MOTOR_OUT = 144
c_2_TIP_MOTOR_IN = 448
c_2_MID_MOTOR_UP = 613
c_2_MID_MOTOR_DOWN = 266
c_2_ROT_MOTOR_RIGHT = 144
c_2_ROT_MOTOR_LEFT = 620

c_3_TIP_MOTOR_OUT = 618
c_3_TIP_MOTOR_IN = 304
c_3_MID_MOTOR_UP = 148
c_3_MID_MOTOR_DOWN = 493
c_3_ROT_MOTOR_RIGHT = 155
c_3_ROT_MOTOR_LEFT = 601

c_4_TIP_MOTOR_OUT = 603
c_4_TIP_MOTOR_IN = 291
c_4_MID_MOTOR_UP = 145
c_4_MID_MOTOR_DOWN = 487
c_4_ROT_MOTOR_RIGHT = 154
c_4_ROT_MOTOR_LEFT = 590

c_5_TIP_MOTOR_OUT = 603
c_5_TIP_MOTOR_IN = 303
c_5_MID_MOTOR_UP = 142
c_5_MID_MOTOR_DOWN = 482
c_5_ROT_MOTOR_RIGHT = 155
c_5_ROT_MOTOR_LEFT = 614

c_L_ARM_TIP_MOTOR_OUT = 609
c_L_ARM_TIP_MOTOR_IN = 153
c_L_ARM_MID_MOTOR_OUT = 92
c_L_ARM_MID_MOTOR_IN = 544
c_L_ARM_ROT_MOTOR_UP = 118
c_L_ARM_ROT_MOTOR_DOWN = 574

c_R_ARM_TIP_MOTOR_OUT = 146
c_R_ARM_TIP_MOTOR_IN = 603
c_R_ARM_MID_MOTOR_OUT = 569
c_R_ARM_MID_MOTOR_IN = 135
c_R_ARM_ROT_MOTOR_UP = 628
c_R_ARM_ROT_MOTOR_DOWN = 110



# same regardless of side so no need to l/r differentiate
TIP_MOTOR_OUT_ANGLE = 180
TIP_MOTOR_IN_ANGLE = 60
MID_MOTOR_UP_ANGLE = 180
MID_MOTOR_DOWN_ANGLE = 45
ROT_MOTOR_RIGHT_ANGLE = 180
ROT_MOTOR_LEFT_ANGLE = 0

ARM_TIP_MOTOR_OUT_ANGLE = 180
ARM_TIP_MOTOR_IN_ANGLE = 0
ARM_MID_MOTOR_OUT_ANGLE = 180
ARM_MID_MOTOR_IN_ANGLE = 0
ARM_ROT_MOTOR_UP_ANGLE = 180
ARM_ROT_MOTOR_DOWN_ANGLE = 0

# rotation motor constants
ROTATOR_MOTOR_LEFT = 424
ROTATOR_MOTOR_RIGHT = 217
ROTATOR_LEFT_ANGLE = 45
ROTATOR_RIGHT_ANGLE = 135

TIP_MOTOR = 1
MID_MOTOR = 2
ROT_MOTOR = 3

LEFT = 1
RIGHT = 2

# error codes and coding constants
SUCCESS = 0
INV_PARAM = -1
ILLEGAL_MOVE = -2

LEG_0 = 0
LEG_1 = 1
LEG_2 = 2
LEG_3 = 3
LEG_4 = 4
LEG_5 = 5

ARM_L = 6
ARM_R = 7

# helper functions
# returns slope given two points
def slope(x1, y1, x2, y2):
    return (y2 - y1) / (x2 - x1)


def intercept(x2, y2, slope):
    return y2 - slope * x2


def linear_map(x1, y1, x2, y2, x_in_val):
    m = slope(x1, y1, x2, y2)
    b = intercept(x2, y2, m)
    return x_in_val * m + b

def get_front_from_direction(direction):
    if(direction == 0):
        return "5-0"
    elif(direction == 60):
        return "0-1"
    elif(direction == 120):
        return "1-2"
    elif(direction == 180):
        return "2-3"
    elif(direction == 240):
        return "3-4"
    elif(direction == 300):
        return "4-5"
    else:
        return "5-0"

# NOTE: these values are ANGLES not raw pwms
class Leg_Position(object):
    def __init__(self, tip_motor, mid_motor, rot_motor):
        self.tip_motor = tip_motor
        self.mid_motor = mid_motor
        self.rot_motor = rot_motor

    def __str__(self):
        return "TIP : " + str(self.tip_motor) + " MID : " + str(self.mid_motor) + " ROT : " + str(self.rot_motor)

# a leg object should be able to associate itself with specific channels
# on a specific i2c interface address and then control all parts of the
# leg. It should be able to set each join according to a degree, max, min,
# or a value from 0-100 in terms of percent of maximum
class Leg(object):
    def __init__(self, uid, pwm, tip_channel, mid_channel, rot_channel, leg_num):
        self.uid = uid
        self.pwm = pwm
        self.tip_channel = tip_channel
        self.mid_channel = mid_channel
        self.rot_channel = rot_channel
        # now, assign the correct constants
        if leg_num == LEG_0:
            self.TIP_MOTOR_OUT      = c_0_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_0_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_0_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_0_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_0_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_0_ROT_MOTOR_LEFT
        elif leg_num == LEG_1:
            self.TIP_MOTOR_OUT      = c_1_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_1_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_1_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_1_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_1_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_1_ROT_MOTOR_LEFT
        elif leg_num == LEG_2:
            self.TIP_MOTOR_OUT      = c_2_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_2_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_2_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_2_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_2_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_2_ROT_MOTOR_LEFT
        elif leg_num == LEG_3:
            self.TIP_MOTOR_OUT      = c_3_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_3_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_3_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_3_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_3_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_3_ROT_MOTOR_LEFT
        elif leg_num == LEG_4:
            self.TIP_MOTOR_OUT      = c_4_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_4_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_4_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_4_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_4_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_4_ROT_MOTOR_LEFT
        elif leg_num == LEG_5:
            self.TIP_MOTOR_OUT      = c_5_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_5_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_5_MID_MOTOR_UP
            self.MID_MOTOR_DOWN     = c_5_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT    = c_5_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT     = c_5_ROT_MOTOR_LEFT
        elif leg_num == ARM_R:
            self.TIP_MOTOR_OUT      = c_R_ARM_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_R_ARM_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_R_ARM_MID_MOTOR_OUT
            self.MID_MOTOR_DOWN     = c_R_ARM_MID_MOTOR_IN
            self.ROT_MOTOR_RIGHT    = c_R_ARM_ROT_MOTOR_UP
            self.ROT_MOTOR_LEFT     = c_R_ARM_ROT_MOTOR_DOWN
        elif leg_num == ARM_L:
            self.TIP_MOTOR_OUT      = c_L_ARM_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN       = c_L_ARM_TIP_MOTOR_IN
            self.MID_MOTOR_UP       = c_L_ARM_MID_MOTOR_OUT
            self.MID_MOTOR_DOWN     = c_L_ARM_MID_MOTOR_IN
            self.ROT_MOTOR_RIGHT    = c_L_ARM_ROT_MOTOR_UP
            self.ROT_MOTOR_LEFT     = c_L_ARM_ROT_MOTOR_DOWN

        if(leg_num == ARM_L or leg_num == ARM_R):
            self.TIP_MOTOR_OUT_ANGLE = ARM_TIP_MOTOR_OUT_ANGLE
            self.TIP_MOTOR_IN_ANGLE = ARM_TIP_MOTOR_IN_ANGLE
            self.MID_MOTOR_UP_ANGLE = ARM_MID_MOTOR_OUT_ANGLE
            self.MID_MOTOR_DOWN_ANGLE = ARM_MID_MOTOR_IN_ANGLE
            self.ROT_MOTOR_RIGHT_ANGLE = ARM_ROT_MOTOR_UP_ANGLE
            self.ROT_MOTOR_LEFT_ANGLE = ARM_ROT_MOTOR_DOWN_ANGLE
        else:
            self.TIP_MOTOR_OUT_ANGLE = TIP_MOTOR_OUT_ANGLE
            self.TIP_MOTOR_IN_ANGLE = TIP_MOTOR_IN_ANGLE
            self.MID_MOTOR_UP_ANGLE = MID_MOTOR_UP_ANGLE
            self.MID_MOTOR_DOWN_ANGLE = MID_MOTOR_DOWN_ANGLE
            self.ROT_MOTOR_RIGHT_ANGLE = ROT_MOTOR_RIGHT_ANGLE
            self.ROT_MOTOR_LEFT_ANGLE = ROT_MOTOR_LEFT_ANGLE


        # set initial values that are not given
        self.tip_motor = -1
        self.mid_motor = -1
        self.rot_motor = -1
        self.tip_motor_angle = -1
        self.mid_motor_angle = -1
        self.rot_motor_angle = -1
        if(leg_num == ARM_L or leg_num == ARM_R):
            self.set_leg_position(TORSO_ARM_TABLE["NEUTRAL"])
        else:
            self.set_angle(45, TIP_MOTOR)
            self.set_angle(180, MID_MOTOR)
            self.set_angle(90, ROT_MOTOR)


    def print_self(self):
        print("leg uid : " + str(self.uid) + " ===========================")
        print("on channels : " + str(self.tip_channel) + " " + str(self.mid_channel) + " " + str(self.rot_channel))
        print("tip_motor pwm : " + str(self.tip_motor) + " angle : " + str(self.pwm_to_angle(self.tip_motor, TIP_MOTOR)))
        print("mid_motor pwm : " + str(self.mid_motor) + " angle : " + str(self.pwm_to_angle(self.mid_motor, MID_MOTOR)))
        print("rot_motor pwm : " + str(self.rot_motor) + " angle : " + str(self.pwm_to_angle(self.rot_motor, ROT_MOTOR)))
        # mock function useful for testing when not in the lab. change all instances
        # of pwm.set_pwm to pwm.set_pwm to use the real function
        # Use the pwm.pwn for real, robot usage

    def angle_to_pwm(self, angle, motor):
        if motor == TIP_MOTOR:
            return linear_map(self.TIP_MOTOR_OUT_ANGLE, self.TIP_MOTOR_OUT, self.TIP_MOTOR_IN_ANGLE, self.TIP_MOTOR_IN,
                              angle)
        elif motor == MID_MOTOR:
            return linear_map(self.MID_MOTOR_UP_ANGLE, self.MID_MOTOR_UP, self.MID_MOTOR_DOWN_ANGLE, self.MID_MOTOR_DOWN,
                              angle)
        elif motor == ROT_MOTOR:
            return linear_map(self.ROT_MOTOR_RIGHT_ANGLE, self.ROT_MOTOR_RIGHT, self.ROT_MOTOR_LEFT_ANGLE, self.ROT_MOTOR_LEFT,
                              angle)
        else:
            return INV_PARAM


    def percent_to_angle(self, percent, motor):
        # maps 0-100 to each motor's min and max angle values
        if motor == TIP_MOTOR:
            return linear_map(100, self.TIP_MOTOR_OUT_ANGLE, 0, self.TIP_MOTOR_IN_ANGLE, percent)
        elif motor == MID_MOTOR:
            return linear_map(100, self.MID_MOTOR_UP_ANGLE, 0, self.MID_MOTOR_DOWN_ANGLE, percent)
        elif motor == ROT_MOTOR:
            return linear_map(100, self.ROT_MOTOR_RIGHT_ANGLE, 0, self.ROT_MOTOR_LEFT_ANGLE, percent)
        else:
            return INV_PARAM


    def pwm_to_angle(self, pwm, motor):
        if motor == TIP_MOTOR:
            return linear_map(self.TIP_MOTOR_OUT, self.TIP_MOTOR_OUT_ANGLE, self.TIP_MOTOR_IN, self.TIP_MOTOR_IN_ANGLE, pwm)
        elif motor == MID_MOTOR:
            return linear_map(self.MID_MOTOR_UP, self.MID_MOTOR_UP_ANGLE, self.MID_MOTOR_DOWN, self.MID_MOTOR_DOWN_ANGLE,
                              pwm)
        elif motor == ROT_MOTOR:
            return linear_map(self.ROT_MOTOR_RIGHT, self.ROT_MOTOR_RIGHT_ANGLE, self.ROT_MOTOR_LEFT, self.ROT_MOTOR_LEFT_ANGLE,
                              pwm)
        else:
            return INV_PARAM


    def set_percent(self, percent, motor):
        # convert and pass off to set_angle
        self.set_angle(self.percent_to_angle(percent, motor), motor)

    def set_leg_position(self, leg_position):
        self.set_angle(leg_position.tip_motor, TIP_MOTOR)
        self.set_angle(leg_position.mid_motor, MID_MOTOR)
        self.set_angle(leg_position.rot_motor, ROT_MOTOR)

    def set_leg_position_up(self, leg_position):
        self.set_angle(leg_position.tip_motor, TIP_MOTOR)
        self.set_angle(leg_position.mid_motor + 45, MID_MOTOR)
        self.set_angle(leg_position.rot_motor, ROT_MOTOR)

    def set_angle(self, angle, motor):
        # sanity checking for each motor
        if motor == TIP_MOTOR:
            # get pwm val
            pwm_val = int(self.angle_to_pwm(angle, motor))

            # safety check
            upper = max(self.TIP_MOTOR_IN, self.TIP_MOTOR_OUT)
            lower = min(self.TIP_MOTOR_IN, self.TIP_MOTOR_OUT)

            if pwm_val < lower:
                pwm_val = lower

            elif pwm_val > upper:
                pwm_val = upper

            # do the write out and update internal value
            self.tip_motor = pwm_val
            self.tip_motor_angle = angle
            self.pwm.set_pwm(self.tip_channel, 0, pwm_val)

        elif motor == MID_MOTOR:
            # get pwm val
            pwm_val = int(self.angle_to_pwm(angle, motor))

            # safety check
            upper = max(self.MID_MOTOR_UP, self.MID_MOTOR_DOWN)
            lower = min(self.MID_MOTOR_UP, self.MID_MOTOR_DOWN)
            if pwm_val < lower:
                pwm_val = lower

            elif pwm_val > upper:
                pwm_val = upper

            # do the write out and update internal value
            self.mid_motor = pwm_val
            self.mid_motor_angle = angle
            self.pwm.set_pwm(self.mid_channel, 0, pwm_val)

        elif motor == ROT_MOTOR:
            # get pwm val
            pwm_val = int(self.angle_to_pwm(angle, motor))

            # safety check
            upper = max(self.ROT_MOTOR_RIGHT, self.ROT_MOTOR_LEFT)
            lower = min(self.ROT_MOTOR_RIGHT, self.ROT_MOTOR_LEFT)
            if pwm_val < lower:
                pwm_val = lower

            elif pwm_val > upper:
                pwm_val = upper

            # do the write out
            self.rot_motor = pwm_val
            self.rot_motor_angle = angle
            self.pwm.set_pwm(self.rot_channel, 0, pwm_val)

# speed options: this is just the time it waits betweeen moves
PLAID_SPEED = .1
NORMAL = .2
SLOW = .4
DEBUG = 5

class Hex_Walker(object):
    def __init__(self, rf_leg, rm_leg, rr_leg, lr_leg, lm_leg, lf_leg):
        # this is an initial array that serves as a permanent holder
        self.leg0 = rf_leg
        self.leg1 = rm_leg
        self.leg2 = rr_leg
        self.leg3 = lr_leg
        self.leg4 = lm_leg
        self.leg5 = lf_leg

        # now, we assign meaningful values in order to define the "front"
        self.front = "5-0"
        self.lf_leg = lf_leg
        self.lm_leg = lm_leg
        self.lr_leg = lr_leg
        self.rf_leg = rf_leg
        self.rm_leg = rm_leg
        self.rr_leg = rr_leg

        # create the lists of leg combinations that would be useful
        self.all_legs = [rf_leg, rm_leg, rr_leg, lr_leg, lm_leg, lf_leg]
        self.left_legs = [lf_leg, lm_leg, lr_leg]
        self.right_legs = [rf_leg, rm_leg, rr_leg]
        self.left_triangle = [lf_leg, rm_leg, lr_leg]
        self.right_triangle = [rf_leg, lm_leg, rr_leg]
        self.front_legs = [lf_leg, rf_leg]
        self.mid_legs = [lm_leg, rm_leg]
        self.rear_legs = [lr_leg, rr_leg]

        # set operating mode
        self.current_pos = NORMAL_NEUTRAL
        self.speed = NORMAL
        self.front = "5-0"
        # set all legs to neutral
        self.set_hex_walker_position(TALL_NEUTRAL)

    def print_self(self):
        print("speed: " + str(self.speed) + " || self.current_pos: " + str(self.current_pos) + " || self.front: " + self.front)
        for leg in self.all_legs:
            leg.print_self()

    def set_speed(self, new_speed):
        self.speed = new_speed

# this function will change the front from being between the "5-0" legs to being
# between any two legs. The key is "(leg on left)-(leg on right)"
    def set_new_front(self, new_front):
        cp = self.current_pos
        if(cp != TALL_NEUTRAL and cp != NORMAL_NEUTRAL and cp != CROUCH_NEUTRAL):
            print("Cannot change front while not in the neutral position")
            return ILLEGAL_MOVE
        
        # check for which side should be the front and re-assign the legs
        # accordingly
        if( new_front == "0-1" ):
            self.rf_leg = self.leg1
            self.rm_leg = self.leg2
            self.rr_leg = self.leg3
            self.lr_leg = self.leg4
            self.lm_leg = self.leg5
            self.lf_leg = self.leg0
            self.front = new_front
            return SUCCESS

        elif( new_front == "1-2" ):
            self.rf_leg = self.leg2
            self.rm_leg = self.leg3
            self.rr_leg = self.leg4
            self.lr_leg = self.leg5
            self.lm_leg = self.leg0
            self.lf_leg = self.leg1
            self.front = new_front
            return SUCCESS

        elif( new_front == "2-3" ):
            self.rf_leg = self.leg3
            self.rm_leg = self.leg4
            self.rr_leg = self.leg5
            self.lr_leg = self.leg0
            self.lm_leg = self.leg1
            self.lf_leg = self.leg2
            self.front = new_front
            return SUCCESS

        elif( new_front == "3-4" ):
            self.rf_leg = self.leg4
            self.rm_leg = self.leg5
            self.rr_leg = self.leg0
            self.lr_leg = self.leg1
            self.lm_leg = self.leg2
            self.lf_leg = self.leg3
            self.front = new_front
            return SUCCESS

        elif( new_front == "4-5" ):
            self.rf_leg = self.leg5
            self.rm_leg = self.leg0
            self.rr_leg = self.leg1
            self.lr_leg = self.leg2
            self.lm_leg = self.leg3
            self.lf_leg = self.leg4
            self.front = new_front
            return SUCCESS

        elif( new_front == "5-0" ):
            self.rf_leg = self.leg0
            self.rm_leg = self.leg1
            self.rr_leg = self.leg2
            self.lr_leg = self.leg3
            self.lm_leg = self.leg4
            self.lf_leg = self.leg5
            self.front = new_front
            return SUCCESS

        else:
            print("invalid front specified") 
            return INV_PARAM

    def do_move_set(self, hex_walker_position_list):
        for next_pos in hex_walker_position_list:
            if next_pos in HEX_WALKER_POSITIONS[self.current_pos].safe_moves:
                self.set_hex_walker_position(next_pos)
                time.sleep(self.speed)
            else:
                print("invalid move set")
                return ILLEGAL_MOVE
        return SUCCESS
    
    # torso movement functions
    def walk(self, num_steps, direction):
        
        self.set_new_front(get_front_from_direction(direction))
        print("dir: " + get_front_from_direction(direction))
        
        # start walk by lifting legs
        self.set_hex_walker_position(TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL)
        # define positions to go through to get steps from a neutral legs up
        left_step = [
        TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
        TALL_TRI_RIGHT_BACK_LEFT_FORWARD,
        TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
        TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL ]
        
        right_step = [
        TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
        TALL_TRI_RIGHT_FORWARD_LEFT_BACK,
        TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
        TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL ]
        
        last_step = "right"

        for i in range (0, num_steps):
            if(last_step == "right"):
                self.do_move_set(left_step)
                last_step = "left"
            elif(last_step == "left"):
                self.do_move_set(right_step)
                last_step = "right"
        #cleanup
        self.set_hex_walker_position(TALL_NEUTRAL)
        self.set_new_front("5-0")

    def rotate(self, num_steps, direction):
        
        # start rotate by lifting legs
        self.set_hex_walker_position(TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL)
        # define positions to go through to get steps from neutral legs up
        go_left_right_step = [
        TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
        TALL_TRI_RIGHT_RIGHT_LEFT_LEFT,
        TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
        TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL]

        go_left_left_step = [
        TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
        TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
        TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
        TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL]

        go_right_right_step = [
        TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
        TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
        TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
        TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL]

        go_right_left_step = [
        TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
        TALL_TRI_RIGHT_RIGHT_LEFT_LEFT,
        TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
        TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL]

        if(direction == RIGHT):
            left_step = go_right_left_step
            right_step = go_right_right_step
        if(direction == LEFT):
            left_step = go_left_left_step
            right_step = go_left_right_step

        last_step = "right"
        for i in range (0, num_steps):
            if(last_step == "right"):
                self.do_move_set(left_step)
                last_step = "left"
            elif(last_step == "left"):
                self.do_move_set(right_step)
                last_step = "right"
        #cleanup
        self.set_hex_walker_position(TALL_NEUTRAL)

    def leg_wave(self, direction, speed, repetitions):
        for i in range(0, repetitions):
            if(direction == RIGHT):
                for leg in self.all_legs:
                    leg.set_leg_position(MISC_TABLE["PULL_UP"])
                    time.sleep(speed)
                    leg.set_leg_position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"])
            if(direction == LEFT):
                for i in range(len(self.all_legs)-1, -1, -1):
                    self.all_legs[i].set_leg_position(MISC_TABLE["PULL_UP"])
                    time.sleep(speed)
                    self.all_legs[i].set_leg_position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"])

    def bounce(self, wait, repetitions):
        for i in range(0, repetitions):
            self.set_hex_walker_position(TALL_TRI_BOUNCE_DOWN)
            time.sleep(wait)
            self.set_hex_walker_position(TALL_NEUTRAL)
            time.sleep(wait)

    def do_nothing(self):
        self.set_hex_walker_position(TALL_NEUTRAL)

# NOTE: the functinos set_hex_walker_position and do_set_hex_walker_position are similar but one takes in a raw position and the other uses the defined table AND updates the current position. Using the do
# version skips this state-updating and so it can be useful for testing

# NOTE: this function should not be called from external code (except testing) because it might not be safe
    def set_hex_walker_position(self, hex_walker_position_number):
        if(HW_MOVE_DEBUG):
            print("current position is : " + HEX_WALKER_POSITIONS[self.current_pos].description + ", moving to position: " + HEX_WALKER_POSITIONS[hex_walker_position_number].description)
        self.current_pos = hex_walker_position_number
        self.do_set_hex_walker_position(HEX_WALKER_POSITIONS[hex_walker_position_number])

# NOTE: this function should not be called from external code (except testing) because it might not be safe
    def do_set_hex_walker_position(self, hex_walker_position):
        self.rf_leg.set_leg_position(hex_walker_position.rf_pos)
        self.rm_leg.set_leg_position(hex_walker_position.rm_pos)
        self.rr_leg.set_leg_position(hex_walker_position.rr_pos)
        self.lr_leg.set_leg_position(hex_walker_position.lr_pos)
        self.lm_leg.set_leg_position(hex_walker_position.lm_pos)
        self.lf_leg.set_leg_position(hex_walker_position.lf_pos)

class Rotator(object):
    def __init__(self, uid, pwm, channel):
        self.uid = uid
        self.pwm = pwm
        self.channel = channel
        self.pwm_val = -1
        self.ROTATOR_MOTOR_LEFT  = ROTATOR_MOTOR_LEFT
        self.ROTATOR_MOTOR_RIGHT = ROTATOR_MOTOR_RIGHT
        self.ROTATOR_LEFT_ANGLE = ROTATOR_LEFT_ANGLE
        self.ROTATOR_RIGHT_ANGLE = ROTATOR_RIGHT_ANGLE
        self.set_angle(90)

    def angle_to_pwm(self, angle):
        return linear_map(self.ROTATOR_LEFT_ANGLE, self.ROTATOR_MOTOR_LEFT, self.ROTATOR_RIGHT_ANGLE, self.ROTATOR_MOTOR_RIGHT, angle)

    def set_angle(self, angle):
        pwm_val = int(self.angle_to_pwm(angle))
        # safety check
        upper = max(self.ROTATOR_MOTOR_LEFT, self.ROTATOR_MOTOR_RIGHT)
        lower = min(self.ROTATOR_MOTOR_LEFT, self.ROTATOR_MOTOR_RIGHT)

        if pwm_val < lower:
            pwm_val = lower

        elif pwm_val > upper:
            pwm_val = upper

        self.pwm_val = pwm_val
        self.pwm.set_pwm(self.channel, 0, pwm_val)

class Robot_Torso(object):
    def __init__(self, right_arm, left_arm, rotator):
        self.right_arm = right_arm
        self.left_arm = left_arm
        self.rotator = rotator
        self.current_position = TORSO_NEUTRAL
        self.set_torso_position(TORSO_NEUTRAL, 90)

    def set_torso_position(self, torso_position_number, rotation):
        self.current_position = torso_position_number
        self.do_set_torso_position(TORSO_POSITIONS[torso_position_number], rotation)

    def do_set_torso_position(self, torso_position, rotation):
        self.right_arm.set_leg_position(torso_position.right_arm)
        self.left_arm.set_leg_position(torso_position.left_arm)
        self.rotator.set_angle(rotation)

    def do_moveset(self, positions, rotations, sleeps, repetitions):
        for j in range(0, repetitions):
            for i in range(0, len(positions)):
                self.set_torso_position(positions[i], rotations[i])
                time.sleep(sleeps[i])

    def set_torso_rotation(self, rotation):
        self.rotator.set_angle(rotation)

    # torso movement functions
    def monkey(self, repetitions):
        moves = []
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        moves.append(TORSO_MONKEY_RIGHT_UP)
        moves.append(TORSO_MONKEY_LEFT_UP)
        rotations = [45, 45, 45, 45, 45, 45, 45, 45, 135, 135, 135, 135, 135, 135, 135,135]
        sleeps =    [.1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1]
        self.do_moveset(moves, rotations, sleeps, repetitions)
        self.set_torso_position(TORSO_NEUTRAL, 90)
    
    def look(self):
        self.set_torso_position(TORSO_LOOKING, 90)

    def point(self, direction, duration):
        if(direction == RIGHT):
            self.set_torso_position(TORSO_POINTING_RIGHT, 90)
        else:
            self.set_torso_position(TORSO_POINTING_LEFT, 90)
        time.sleep(duration)
        self.set_torso_position(TORSO_NEUTRAL, 90)

    def king_kong(self, rotation, repetitions):
        moves = []
        moves.append(TORSO_DANCE_FRONT_LEFT_OUT)
        moves.append(TORSO_DANCE_FRONT_RIGHT_OUT)
        rotations = [rotation, rotation]
        sleeps = [.4, .4]
        self.do_moveset(moves, rotations, sleeps, repetitions)
        self.set_torso_position(TORSO_NEUTRAL, 90)
    
    def hand_shake(self, rotation, repetitions):
        moves = []
        moves.append(TORSO_SHAKE_DOWN)
        moves.append(TORSO_SHAKE_MID)
        moves.append(TORSO_SHAKE_UP)
        moves.append(TORSO_SHAKE_MID)
        rotations = [rotation, rotation, rotation, rotation]
        sleeps = [.1, .1, .1, .1]
        self.do_moveset(moves, rotations, sleeps, repetitions)
        self.set_torso_position(TORSO_NEUTRAL, 90)
    
    def wave(self, rotation, repetitions):
        moves = []
        moves.append(TORSO_WAVE_DOWN)
        moves.append(TORSO_WAVE_UP)
        rotations = [rotation, rotation]
        sleeps = [.4, .4]
        self.do_moveset(moves, rotations, sleeps, repetitions)
        self.set_torso_position(TORSO_NEUTRAL, 90)

    def neutral_rotate(self, direction):
        self.set_torso_position(TORSO_NEUTRAL, direction)

    def do_nothing(self):
        self.set_torso_position(TORSO_NEUTRAL, 90)
