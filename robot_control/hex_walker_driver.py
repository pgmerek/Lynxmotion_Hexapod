from __future__ import division
import time
from hex_walker_data import *
from leg_data import *
# uncomment if working with the actual robot
import Adafruit_PCA9685

#Extraneous
HW_MOVE_DEBUG = 1 #toggle 0/1 to turn debug prints on/off

# all constants related to the hex_walker legs
L_TIP_MOTOR_OUT = 600
L_TIP_MOTOR_IN = 300
L_MID_MOTOR_UP = 120
L_MID_MOTOR_DOWN = 450
L_ROT_MOTOR_RIGHT = 140
L_ROT_MOTOR_LEFT = 590

# right legs are left legs but mirrored so values are different
# Test before using
R_TIP_MOTOR_OUT = 150  
R_TIP_MOTOR_IN = 490
R_MID_MOTOR_UP = 610
R_MID_MOTOR_DOWN = 280 
R_ROT_MOTOR_RIGHT = 150 
R_ROT_MOTOR_LEFT = 610

# same regardless of side so no need to l/r differentiate
TIP_MOTOR_OUT_ANGLE = 180
TIP_MOTOR_IN_ANGLE = 45
MID_MOTOR_UP_ANGLE = 180
MID_MOTOR_DOWN_ANGLE = 45
ROT_MOTOR_RIGHT_ANGLE = 180
ROT_MOTOR_LEFT_ANGLE = 0

TIP_MOTOR = 1
MID_MOTOR = 2
ROT_MOTOR = 3

LEFT = 1
RIGHT = 2

# error codes and coding constants
SUCCESS = 0
INV_PARAM = -1
ILLEGAL_MOVE = -2

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
    def __init__(self, uid, pwm, tip_channel, mid_channel, rot_channel, side):
        self.uid = uid
        self.pwm = pwm
        self.tip_channel = tip_channel
        self.mid_channel = mid_channel
        self.rot_channel = rot_channel
        # now, assign the correct constants
        if side == LEFT:
            self.TIP_MOTOR_OUT = L_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN = L_TIP_MOTOR_IN
            self.MID_MOTOR_UP = L_MID_MOTOR_UP
            self.MID_MOTOR_DOWN = L_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT = L_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT = L_ROT_MOTOR_LEFT
        elif side == RIGHT:
            self.TIP_MOTOR_OUT = R_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN = R_TIP_MOTOR_IN
            self.MID_MOTOR_UP = R_MID_MOTOR_UP
            self.MID_MOTOR_DOWN = R_MID_MOTOR_DOWN
            self.ROT_MOTOR_RIGHT = R_ROT_MOTOR_RIGHT
            self.ROT_MOTOR_LEFT = R_ROT_MOTOR_LEFT
        # these could just be left global, but make them self.consts for continuity
        self.TIP_MOTOR_IN_ANGLE = TIP_MOTOR_IN_ANGLE
        self.TIP_MOTOR_OUT_ANGLE = TIP_MOTOR_OUT_ANGLE
        self.MID_MOTOR_UP_ANGLE = MID_MOTOR_UP_ANGLE
        self.MID_MOTOR_DOWN_ANGLE = MID_MOTOR_DOWN_ANGLE
        self.ROT_MOTOR_RIGHT_ANGLE = ROT_MOTOR_RIGHT_ANGLE
        self.ROT_MOTOR_LEFT_ANGLE = ROT_MOTOR_LEFT_ANGLE

        # set initial values that are not given
        self.tip_motor = -1
        self.mid_motor = -1
        self.rot_motor = -1
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


    def pwm_set_pwm(self, channel, unknown, value):
        x = 3
        # returns the angle on success. INV_PARAM otherwise


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
            self.pwm_set_pwm(self.tip_channel, 0, pwm_val)

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
            self.pwm_set_pwm(self.mid_channel, 0, pwm_val)

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
            self.pwm_set_pwm(self.rot_channel, 0, pwm_val)

# speed options: this is just the time it waits betweeen moves
PLAID_SPEED = .1
ULTRA = .3
FAST = .5
NORMAL = 1
SAFE = 2
SLOW = 3
SLOOOOOOOOOOW = 5
SLOOOOOOOOOOOOOOOOOOW = 10

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
        for leg in self.all_legs:
            leg.set_leg_position(NORMAL_TRI_ROTATION_TABLE["NEUTRAL"])

    def print_self(self):
        print("speed: " + str(self.speed) + " || self.current_pos: " + str(self.current_pos) + " || self.front: " + self.front)
        for leg in self.all_legs:
            leg.print_self()

    def set_speed(self, new_speed):
        self.speed = new_speed

# this function will change the front from being between the "5-0" legs to being
# between any two legs. The key is "(leg on left)-(leg on right)"
    def set_new_front(self, new_front):
        if(self.current_pos != NORMAL_NEUTRAL):
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
                if(HW_MOVE_DEBUG):
                    self.print_self()
                time.sleep(self.speed)
            else:
                print("invalid move set")
                return ILLEGAL_MOVE
        return SUCCESS

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

    
