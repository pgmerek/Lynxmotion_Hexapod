
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
        "NEUTRAL"           : Leg_Position(90, 90, 90),
        "UP_NEUTRAL"        : Leg_Position(90, 135, 90),
        
        "RIGHT"             : Leg_Position(95, 90, 120),
        "UP_RIGHT"          : Leg_Position(95, 135, 120),
        
        "LEFT"              : Leg_Position(95, 90, 60),
        "UP_LEFT"           : Leg_Position(95, 135, 60)

   }

# table to be used when the robot is trying to move in a "normal" way (moving with two legs forward)
# tip motor, mid motor, rot motor
NORMAL_TRI_MOVEMENT_TABLE = {
#all the positions for the front and back legs
        "NEUTRAL"               : Leg_Position(120, 90, 90),
        "UP_NEUTRAL"            : Leg_Position(120, 135, 90),
      
        "CORN_OUT"              : Leg_Position(135, 105, 75),
        "CORN_IN"               : Leg_Position(85, 80, 125),

        "CORN_UP_OUT"           : Leg_Position(135, 150, 75),
        "CORN_UP_IN"            : Leg_Position(85, 125, 125),

#now all of the positions for the side legs
        "SIDE_RIGHT"            : Leg_Position(80, 90, 115),
        "SIDE_LEFT"             : Leg_Position(80, 90, 65),

        "SIDE_UP_RIGHT"         : Leg_Position(80, 135, 115),
        "SIDE_UP_LEFT"          : Leg_Position(80, 135, 65)
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
# table to be used when the robot is trying to rotate in place
CROUCH_TRI_ROTATION_TABLE = {
        "NEUTRAL"           : Leg_Position(45, 135, 90),
        "UP_NEUTRAL"        : Leg_Position(45, 180, 90),
        
        "RIGHT"             : Leg_Position(50, 130, 120),
        "UP_RIGHT"          : Leg_Position(50, 180, 120),
        
        "LEFT"              : Leg_Position(50, 130, 60),
        "UP_LEFT"           : Leg_Position(50, 180, 60)

}

CROUCH_TRI_MOVEMENT_TABLE = {
        "NEUTRAL"               : Leg_Position(45, 135, 90),
        "UP_NEUTRAL"            : Leg_Position(45, 180, 90),
      
        "CORN_OUT_LEFT"         : Leg_Position(80, 125, 85),
        "CORN_OUT_RIGHT"        : Leg_Position(80, 125, 95),

        "CORN_UP_OUT_LEFT"      : Leg_Position(85, 175, 85),
        "CORN_UP_OUT_RIGHT"     : Leg_Position(85, 175, 95),

        "SIDE_RIGHT"            : Leg_Position(70, 125, 105),
        "SIDE_LEFT"             : Leg_Position(70, 125, 75),

        "SIDE_UP_RIGHT"         : Leg_Position(70, 170, 95),
        "SIDE_UP_LEFT"          : Leg_Position(70, 170, 85)
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
        "NEUTRAL"           : Leg_Position(120, 45, 90),
        "UP_NEUTRAL"        : Leg_Position(45, 90, 90),
        
        "RIGHT"             : Leg_Position(120, 45, 120),
        "UP_RIGHT"          : Leg_Position(45, 90, 120),
        
        "LEFT"              : Leg_Position(120, 45, 60),
        "UP_LEFT"           : Leg_Position(45, 90, 60)

}

TALL_TRI_MOVEMENT_TABLE = {
        "NEUTRAL"               : Leg_Position(120, 45, 90),
        "UP_NEUTRAL"            : Leg_Position(45, 90, 90),
      
        "CORN_OUT_LEFT"         : Leg_Position(125, 52, 80),
        "CORN_OUT_RIGHT"        : Leg_Position(125, 52, 100),

        "CORN_UP_OUT_LEFT"      : Leg_Position(60, 90, 80),
        "CORN_UP_OUT_RIGHT"     : Leg_Position(60, 90, 100),

        "SIDE_RIGHT"            : Leg_Position(115, 60, 130),
        "SIDE_LEFT"             : Leg_Position(115, 60, 50),

        "SIDE_UP_RIGHT"         : Leg_Position(60, 80, 130),
        "SIDE_UP_LEFT"          : Leg_Position(60, 80, 50)
}

# There's no center in because the mid motor is limited to 45 degrees 
TALL_TRI_SIDE_MOVEMENT_TABLE = {
        "NEUTRAL": Leg_Position(120, 45, 90),
        "UP_NEUTRAL": Leg_Position(45, 90, 90),

        "SIDE_OUT_LEFT": Leg_Position(140, 48, 83),
        "SIDE_OUT_RIGHT": Leg_Position(120, 50, 125),

        "SIDE_UP_OUT_LEFT": Leg_Position(110, 68, 83),
        "SIDE_UP_OUT_RIGHT": Leg_Position(90, 70, 125),

        "CENTER_OUT": Leg_Position(130, 50, 90),
         
        "CENTER_UP_OUT": Leg_Position(100, 70, 90),
        }


# misc table
MISC_TABLE = {
        "BOUNCE"            : Leg_Position(90, 75, 90),
        "PULL_UP"           : Leg_Position(60, 75, 90)

   }


