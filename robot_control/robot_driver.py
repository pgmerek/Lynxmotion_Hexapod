from __future__ import division
import time

# uncomment if working with the actual robot
# import Adafruit_PCA9685


# all constants related to the robot legs
L_TIP_MOTOR_OUT = 600
L_TIP_MOTOR_IN = 300
L_MID_MOTOR_UP = 120
L_MID_MOTOR_DOWN = 450
L_ROT_MOTOR_FOR = 140
L_ROT_MOTOR_BACK = 590

# right legs are left legs but mirrored so values are different
# Test before using
R_TIP_MOTOR_OUT = 300
R_TIP_MOTOR_IN = 600
R_MID_MOTOR_UP = 450
R_MID_MOTOR_DOWN = 120
R_ROT_MOTOR_FOR = 590
R_ROT_MOTOR_BACK = 140

# same regardless of side so no need to l/r differentiate
TIP_MOTOR_OUT_ANGLE = 180
TIP_MOTOR_IN_ANGLE = 45
MID_MOTOR_UP_ANGLE = 180
MID_MOTOR_DOWN_ANGLE = 45
ROT_MOTOR_FOR_ANGLE = 180
ROT_MOTOR_BACK_ANGLE = 0

TIP_MOTOR = 1
MID_MOTOR = 2
ROT_MOTOR = 3

LEFT = 1
RIGHT = 2

# error codes and coding constants
SUCCESS = 0
INV_PARAM = -1


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


# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse = int(pulse)
    pulse_length = 1000000  # 1,000,000 us per second
    pulse_length //= 60  # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096  # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)


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
            self.ROT_MOTOR_FOR = L_ROT_MOTOR_FOR
            self.ROT_MOTOR_BACK = L_ROT_MOTOR_BACK
        elif side == RIGHT:
            self.TIP_MOTOR_OUT = R_TIP_MOTOR_OUT
            self.TIP_MOTOR_IN = R_TIP_MOTOR_IN
            self.MID_MOTOR_UP = R_MID_MOTOR_UP
            self.MID_MOTOR_DOWN = R_MID_MOTOR_DOWN
            self.ROT_MOTOR_FOR = R_ROT_MOTOR_FOR
            self.ROT_MOTOR_BACK = R_ROT_MOTOR_BACK
        # these could just be left global, but make them self.consts for continuity
        self.TIP_MOTOR_IN_ANGLE = TIP_MOTOR_IN_ANGLE
        self.TIP_MOTOR_OUT_ANGLE = TIP_MOTOR_OUT_ANGLE
        self.MID_MOTOR_UP_ANGLE = MID_MOTOR_UP_ANGLE
        self.MID_MOTOR_DOWN_ANGLE = MID_MOTOR_DOWN_ANGLE
        self.ROT_MOTOR_FOR_ANGLE = ROT_MOTOR_FOR_ANGLE
        self.ROT_MOTOR_BACK_ANGLE = ROT_MOTOR_BACK_ANGLE

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
    # of pwm_set_pwm to pwm.set_pwm to use the real function
    # Use the pwm.pwn for real, robot usage


def pwm_set_pwm(self, channel, unknown, value):
    print("pwm for channel " + str(channel) + " was set as: " + str(value))

    # returns the angle on success. INV_PARAM otherwise


def angle_to_pwm(self, angle, motor):
    if motor == TIP_MOTOR:
        return linear_map(self.TIP_MOTOR_OUT_ANGLE, self.TIP_MOTOR_OUT, self.TIP_MOTOR_IN_ANGLE, self.TIP_MOTOR_IN,
                          angle)
    elif motor == MID_MOTOR:
        return linear_map(self.MID_MOTOR_UP_ANGLE, self.MID_MOTOR_UP, self.MID_MOTOR_DOWN_ANGLE, self.MID_MOTOR_DOWN,
                          angle)
    elif motor == ROT_MOTOR:
        return linear_map(self.ROT_MOTOR_FOR_ANGLE, self.ROT_MOTOR_FOR, self.ROT_MOTOR_BACK_ANGLE, self.ROT_MOTOR_BACK,
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
        return linear_map(100, self.ROT_MOTOR_FOR_ANGLE, 0, self.ROT_MOTOR_BACK_ANGLE, percent)
    else:
        return INV_PARAM


def pwm_to_angle(self, pwm, motor):
    if motor == TIP_MOTOR:
        return linear_map(self.TIP_MOTOR_OUT, self.TIP_MOTOR_OUT_ANGLE, self.TIP_MOTOR_IN, self.TIP_MOTOR_IN_ANGLE, pwm)
    elif motor == MID_MOTOR:
        return linear_map(self.MID_MOTOR_UP, self.MID_MOTOR_UP_ANGLE, self.MID_MOTOR_DOWN, self.MID_MOTOR_DOWN_ANGLE,
                          pwm)
    elif motor == ROT_MOTOR:
        return linear_map(self.ROT_MOTOR_FOR, self.ROT_MOTOR_FOR_ANGLE, self.ROT_MOTOR_BACK, self.ROT_MOTOR_BACK_ANGLE,
                          pwm)
    else:
        return INV_PARAM


def set_percent(self, percent, motor):
    # convert and pass off to set_angle
    self.set_angle(self.percent_to_angle(percent, motor), motor)


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
        upper = max(self.ROT_MOTOR_FOR, self.ROT_MOTOR_BACK)
        lower = min(self.ROT_MOTOR_FOR, self.ROT_MOTOR_BACK)
        if pwm_val < lower:
            pwm_val = lower

        elif pwm_val > upper:
            pwm_val = upper

        # do the write out
        self.rot_motor = pwm_val
        self.pwm_set_pwm(self.rot_channel, 0, pwm_val)

    # end of the main driver here. These are just some extra "useful" commands


def set_neutral(self):
    self.set_angle(135, TIP_MOTOR)
    self.set_angle(45, MID_MOTOR)
    self.set_angle(90, ROT_MOTOR)


def leg_up(self):
    self.set_angle(135, TIP_MOTOR)
    self.set_angle(180, MID_MOTOR)


def leg_down(self):
    self.set_angle(135, TIP_MOTOR)
    self.set_angle(45, MID_MOTOR)


def leg_in(self):
    self.set_angle(45, TIP_MOTOR)
    self.set_angle(45, MID_MOTOR)


def s_for(self):
    self.set_angle(120, ROT_MOTOR)


def s_back(self):
    self.set_angle(60, ROT_MOTOR)


# operation modes
TRIANGLE = 1
ALL_TOGETHER = 2
SEGMENT = 3

# safety options
SAFE = 1
UNSAFE = 2
WILD = 3


class Robot(object):
    def __init__(self, lf_leg, lm_leg, lr_leg, rf_leg, rm_leg, rr_leg):
        self.lf_leg = lf_leg
        self.lm_leg = lm_leg
        self.lr_leg = lr_leg
        self.rf_leg = rf_leg
        self.rm_leg = rm_leg
        self.rr_leg = rr_leg

        # create the lists of leg combinations that would be useful
        self.all_legs = [lf_leg, lm_leg, lr_leg, rf_leg, rm_leg, rr_leg]
        self.left_legs = [lf_leg, lm_leg, lr_leg]
        self.right_legs = [rf_leg, rm_leg, rr_leg]
        self.left_triangle = [lf_leg, rm_leg, lr_leg]
        self.right_triangle = [rf_leg, lm_leg, rr_leg]
        self.front_legs = [lf_leg, rf_leg]
        self.mid_legs = [lm_leg, rm_leg]
        self.rear_legs = [lr_leg, rr_leg]

        # set operating mode
        self.op_mode = TRIANGLE
        self.op_safety = SAFE
        # set all legs to neutral
        for leg in self.all_legs:
            leg.set_neutral()

    def print_self(self):
        for leg in self.all_legs:
            leg.print_self()

    def s_triangle_up(self, side):
        if self.op_mode != TRIANGLE:
            return INV_OP_MODE
        if side == LEFT:
            for leg in self.left_triangle:
                leg.leg_up()
        if side == RIGHT:
            for leg in self.right_triangle:
                leg.leg_up()
        return SUCCESS

    def s_triangle_for(self, side):
        if self.op_mode != TRIANGLE:
            return INV_OP_MODE
        if side == LEFT:
            for leg in self.left_triangle:
                leg.s_for()
        if side == RIGHT:
            for leg in self.right_triangle:
                leg.s_for()
        return SUCCESS

    def s_triangle_back(self, side):
        if self.op_mode != TRIANGLE:
            return INV_OP_MODE
        if side == LEFT:
            for leg in self.left_triangle:
                leg.s_back()
        if side == RIGHT:
            for leg in self.right_triangle:
                leg.s_back()
        return SUCCESS

    def s_triangle_neutral(self, side):
        if self.op_mode != TRIANGLE:
            return INV_OP_MODE
        if side == LEFT:
            for leg in self.left_triangle:
                leg.set_neutral()
        if side == RIGHT:
            for leg in self.right_triangle:
                leg.set_neutral()
        return SUCCESS
