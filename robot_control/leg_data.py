
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
        "NEUTRAL"           : Leg_Position(120, 45, 90),
        "UP_NEUTRAL"        : Leg_Position(120, 90, 90),
        
        "RIGHT"             : Leg_Position(125, 45, 120),
        "UP_RIGHT"          : Leg_Position(125, 90, 120),
        
        "LEFT"              : Leg_Position(125, 45, 60),
        "UP_LEFT"           : Leg_Position(125, 90, 60)


#old        "RIGHT"             : Leg_Position(95, 90, 120),
#old        "NEUTRAL"           : Leg_Position(90, 90, 90),
#old        "LEFT"              : Leg_Position(95, 90, 60),
#old        "UP_RIGHT"          : Leg_Position(95, 135, 120),
#old        "UP_NEUTRAL"        : Leg_Position(90, 135, 90),
#old        "UP_LEFT"           : Leg_Position(95, 135, 60),

   
   }

# table to be used when the robot is trying to move in a "normal" way (moving with two legs forward)
# tip motor, mid motor, rot motor
NORMAL_TRI_MOVEMENT_TABLE = {
#all the positions for the front and back legs
        "NEUTRAL"               : Leg_Position(120, 45, 90),
        "UP_NEUTRAL"            : Leg_Position(120, 90, 90),
      
        "CORN_OUT"              : Leg_Position(135, 105, 75),
        "CORN_IN"               : Leg_Position(85, 80, 125),

        "CORN_UP_OUT"           : Leg_Position(135, 150, 75),
        "CORN_UP_IN"            : Leg_Position(85, 125, 125),

#now all of the positions for the side legs
        "SIDE_RIGHT"            : Leg_Position(80, 90, 115),
        "SIDE_LEFT"             : Leg_Position(80, 90, 65),

        "SIDE_UP_RIGHT"         : Leg_Position(80, 135, 115),
        "SIDE_UP_LEFT"          : Leg_Position(80, 135, 65)


#old        "NEUTRAL"               : Leg_Position(90, 90, 90),
#old        "UP_NEUTRAL"            : Leg_Position(90, 135, 90),
        
#old        "CORN_OUT"              : Leg_Position(135, 105, 75),
#old        "CORN_IN"               : Leg_Position(85, 80, 125),

#old        "CORN_UP_OUT"           : Leg_Position(135, 150, 75),
#old        "CORN_UP_IN"            : Leg_Position(85, 125, 125),

#now all of the positions for the side legs
#old        "SIDE_RIGHT"            : Leg_Position(80, 90, 115),
#old        "SIDE_LEFT"             : Leg_Position(80, 90, 65),

#old        "SIDE_UP_RIGHT"         : Leg_Position(80, 135, 115),
#old        "SIDE_UP_LEFT"          : Leg_Position(80, 135, 65)

        }

# table to be used when the robot is trying to move in a "sideways" way (moving with a single leg forward)
NORMAL_TRI_SIDE_MOVEMENT_TABLE = {
        "NEUTRAL"           : Leg_Position(90, 90, 90),
        "UP_NEUTRAL"        : Leg_Position(90, 135, 90),
        "CORN_LEFT"         : Leg_Position(0, 0, 0),
        "CORN_RIGHT"        : Leg_Position(0, 0, 0),
        "CENT_OUT"          : Leg_Position(0, 0, 0),
        "CENT_IN"           : Leg_Position(0, 0, 0)
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







