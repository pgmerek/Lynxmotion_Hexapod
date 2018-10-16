
# NOTE: these values are ANGLES not raw pwms
class Leg_Position(object):
    def __init__(self, tip_motor, mid_motor, rot_motor):
        self.tip_motor = tip_motor
        self.mid_motor = mid_motor
        self.rot_motor = rot_motor

    def __str__(self):
        return "TIP: " + str(self.tip_motor) + "|| MID : " + str(self.mid_motor) + "|| ROT : " + str(self.rot_motor)

# NOTE: table naming convention is: (standing height)_(gait)_(what type of movement)_TABLE

# table to be used when the robot is trying to rotate in place
NORMAL_TRI_ROTATION_TABLE = {
        "OUT_RIGHT"         : Leg_Position(160, 60, 115),
        "OUT"               : Leg_Position(135, 80, 90),
        "OUT_LEFT"          : Leg_Position(160, 80, 65),
        "RIGHT"             : Leg_Position(95, 90, 120),
        "NEUTRAL"           : Leg_Position(90, 90, 90),
        "LEFT"              : Leg_Position(95, 90, 60),
        "TUCK_RIGHT"        : Leg_Position(75, 85, 115),
        "TUCK"              : Leg_Position(75, 75, 90),
        "TUCK_LEFT"         : Leg_Position(75, 85, 65),
        "UP_OUT_RIGHT"      : Leg_Position(160, 105, 115),
        "UP_OUT"            : Leg_Position(135, 125, 90),
        "UP_OUT_LEFT"       : Leg_Position(160, 125, 65),
        "UP_RIGHT"          : Leg_Position(95, 135, 120),
        "UP_NEUTRAL"        : Leg_Position(90, 135, 90),
        "UP_LEFT"           : Leg_Position(95, 135, 60),
        "UP_TUCK_RIGHT"     : Leg_Position(75, 130, 115),
        "UP_TUCK"           : Leg_Position(75, 120, 90),
        "UP_TUCK_LEFT"      : Leg_Position(75, 130, 65)

        }

# table to be used when the robot is trying to move in a "normal" way (moving with two legs forward)
NORMAL_TRI_MOVEMENT_TABLE = {
        "CORN_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "CORN_OUT"          : Leg_Position(135, 80, 90),
        "CORN_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "CORN_RIGHT"        : Leg_Position(0, 0, 0),
        "CORN_NEUTRAL"      : Leg_Position(90, 90, 90),
        "CORN_LEFT"         : Leg_Position(0, 0, 0),
        "CORN_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "CORN_TUCK"         : Leg_Position(75, 75, 90),
        "CORN_TUCK_LEFT"    : Leg_Position(0, 0, 0),
        
        "CORN_UP_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "CORN_UP_OUT"          : Leg_Position(135, 125, 90),
        "CORN_UP_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "CORN_UP_RIGHT"        : Leg_Position(0, 0, 0),
        "CORN_UP_NEUTRAL"      : Leg_Position(90, 135, 90),
        "CORN_UP_LEFT"         : Leg_Position(0, 0, 0),
        "CORN_UP_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "CORN_UP_TUCK"         : Leg_Position(75, 120, 90),
        "CORN_UP_TUCK_LEFT"    : Leg_Position(0, 0, 0),

        
        "SIDE_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "SIDE_OUT"          : Leg_Position(135, 80, 90),
        "SIDE_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "SIDE_RIGHT"        : Leg_Position(0, 0, 0),
        "SIDE_NEUTRAL"      : Leg_Position(90, 90, 90),
        "SIDE_LEFT"         : Leg_Position(0, 0, 0),
        "SIDE_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "SIDE_TUCK"         : Leg_Position(75, 75, 90),
        "SIDE_TUCK_LEFT"    : Leg_Position(0, 0, 0),

        "SIDE_UP_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "SIDE_UP_OUT"          : Leg_Position(135, 125, 90),
        "SIDE_UP_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "SIDE_UP_RIGHT"        : Leg_Position(0, 0, 0),
        "SIDE_UP_NEUTRAL"      : Leg_Position(90, 135, 90),
        "SIDE_UP_LEFT"         : Leg_Position(0, 0, 0),
        "SIDE_UP_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "SIDE_UP_TUCK"         : Leg_Position(75, 120, 90),
        "SIDE_UP_TUCK_LEFT"    : Leg_Position(0, 0, 0)

        }

# table to be used when the robot is trying to move in a "sideways" way (moving with a single leg forward)
NORMAL_TRI_SIDE_MOVEMENT_TABLE = {
        "CORN_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "CORN_OUT"          : Leg_Position(135, 80, 90),
        "CORN_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "CORN_RIGHT"        : Leg_Position(0, 0, 0),
        "CORN_NEUTRAL"      : Leg_Position(90, 90, 90),
        "CORN_LEFT"         : Leg_Position(0, 0, 0),
        "CORN_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "CORN_TUCK"         : Leg_Position(75, 75, 90),
        "CORN_TUCK_LEFT"    : Leg_Position(0, 0, 0),
        "CENT_OUT_RIGHT"    : Leg_Position(0, 0, 0),
        "CENT_OUT"          : Leg_Position(135, 80, 90),
        "CENT_OUT_LEFT"     : Leg_Position(0, 0, 0),
        "CENT_RIGHT"        : Leg_Position(0, 0, 0),
        "CENT_NEUTRAL"      : Leg_Position(90, 90, 90),
        "CENT_LEFT"         : Leg_Position(0, 0, 0),
        "CENT_TUCK_RIGHT"   : Leg_Position(0, 0, 0),
        "CENT_TUCK"         : Leg_Position(75, 75, 90),
        "CENT_TUCK_LEFT"    : Leg_Position(0, 0, 0)
        }



# this is extra. don't do this until the above is working
CROUCH_TRI_ROTATION_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }

CROUCH_TRI_MOVEMENT_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }


CROUCH_TRI_SIDE_MOVEMENT_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }

TALL_TRI_ROTATION_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }

TALL_TRI_MOVEMENT_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }


TALL_TRI_SIDE_MOVEMENT_TABLE = {
        "OUT_RIGHT"     : Leg_Position(0, 0, 0),
        "OUT"           : Leg_Position(0, 0, 0),
        "OUT_LEFT"      : Leg_Position(0, 0, 0),
        "RIGHT"         : Leg_Position(0, 0, 0),
        "NEUTRAL"       : Leg_Position(0, 0, 0),
        "LEFT"          : Leg_Position(0, 0, 0),
        "TUCK_RIGHT"    : Leg_Position(0, 0, 0),
        "TUCK"          : Leg_Position(0, 0, 0),
        "TUCK_LEFT"     : Leg_Position(0, 0, 0)
        }







