from leg_data import *
from hex_walker_data import *

TORSO_ARM_TABLE = {
        "NEUTRAL"           : Leg_Position(0, 45, 45),
        "ON_HIP"        : Leg_Position(0, 90, 0),
        "UP"             : Leg_Position(90, 0, 180),
        "STRAIGHT_OUT"   : Leg_Position(90,90,90),
        "STRAIGHT_FOR"      :Leg_Position(90,0,90),

        "WAVE_UP"          : Leg_Position(30, 40, 180),
        "WAVE_DOWN"         : Leg_Position(55,120,180),
        
        "HAND_SHAKE_UP"     :Leg_Position(45,10,80),
        "HAND_SHAKE_MID"    :Leg_Position(45, 10, 70),
        "HAND_SHAKE_DOWN"   : Leg_Position(45,10,60),
        
        "JOHNNY_BRAVO_MONKEY_DOWN" : Leg_Position(90,0,80),
        "JOHNNY_BRAVO_MONKEY_UP"  : Leg_Position(90,0,100),

        "BLOCKING_UP"              : Leg_Position(30, 0, 180),
        "BLOCKING_FRONT"           : Leg_Position(30, 0, 90)

   }

class Torso_Position(object):
    def __init__(self, right_arm, left_arm, description):
        self.right_arm = right_arm
        self.left_arm = left_arm
        self.description = description
    
    def __str__(self):
        start_str = "-----------------------Torso position is-------------------------"
        right_arm_string = "right arm: " + str(self.right_arm) + "\n"
        left_arm_string = "left arm: " + str(self.left_arm) + "\n"
        return right_arm_string + left_arm_string + self.description

# relaxed
TORSO_NEUTRAL = 1

# jumping jacks
TORSO_JACK_DOWN = 2
TORSO_JACK_UP = 3

# right hand wave
TORSO_WAVE_DOWN = 4
TORSO_WAVE_UP = 5

# right hand shake
TORSO_SHAKE_DOWN = 6
TORSO_SHAKE_UP = 7

# dancing in front
TORSO_DANCE_FRONT_LEFT_OUT = 8
TORSO_DANCE_FRONT_RIGHT_OUT = 9

# dancing above
TORSO_DANCE_ABOVE_LEFT_UP = 10
TORSO_DANCE_ABOVE_RIGHT_UP = 11

# jognny bravo dance
TORSO_MONKEY_RIGHT_UP = 12
TORSO_MONKEY_LEFT_UP = 13

TORSO_POSITIONS = {
        # 1
        TORSO_NEUTRAL:
        Torso_Position(TORSO_ARM_TABLE["NEUTRAL"],
                        TORSO_ARM_TABLE["NEUTRAL"],
                        "torso is in the neutral position"),
        # 2
        TORSO_JACK_DOWN:
        Torso_Position(TORSO_ARM_TABLE["WAVE_DOWN"],
                        TORSO_ARM_TABLE["WAVE_DOWN"],
                        "jumping jacks (down pos)"),
        
        # 3
        TORSO_JACK_UP:
        Torso_Position(TORSO_ARM_TABLE["WAVE_UP"],
                        TORSO_ARM_TABLE["WAVE_UP"],
                        "jumping jacks (up pos)"),
        # 4
        TORSO_WAVE_DOWN:
        Torso_Position(TORSO_ARM_TABLE["WAVE_DOWN"],
                        TORSO_ARM_TABLE["NEUTRAL"],
                        "waving with the right hand (down pos)"),

        # 5
        TORSO_WAVE_UP:
        Torso_Position(TORSO_ARM_TABLE["WAVE_UP"],
                        TORSO_ARM_TABLE["NEUTRAL"],
                        "waving with the right hand (up pos)"),
        # 6
        TORSO_SHAKE_DOWN:
        Torso_Position(TORSO_ARM_TABLE["HAND_SHAKE_DOWN"],
                        TORSO_ARM_TABLE["NEUTRAL"],
                        "handshaking with the right hand (down pos)"),
        
        # 7
        TORSO_SHAKE_UP:
        Torso_Position(TORSO_ARM_TABLE["HAND_SHAKE_UP"],
                        TORSO_ARM_TABLE["NEUTRAL"],
                        "handshaking with the right hand (up pos)"),
        # 8
        TORSO_DANCE_FRONT_LEFT_OUT:
        Torso_Position(TORSO_ARM_TABLE["BLOCKING_FRONT"],
                        TORSO_ARM_TABLE["STRAIGHT_OUT"],
                        "dance move with left arm out"),
        # 9
        TORSO_DANCE_FRONT_RIGHT_OUT:
        Torso_Position(TORSO_ARM_TABLE["STRAIGHT_OUT"],
                        TORSO_ARM_TABLE["BLOCKING_FRONT"],
                        "dance move with right arm out"),
        # 10
        TORSO_DANCE_ABOVE_LEFT_UP:
        Torso_Position(TORSO_ARM_TABLE["BLOCKING_UP"],
                        TORSO_ARM_TABLE["WAVE_DOWN"],
                        "dance move with left arm above head"),
        
        # 11
        TORSO_DANCE_ABOVE_RIGHT_UP:
        Torso_Position(TORSO_ARM_TABLE["WAVE_DOWN"],
                        TORSO_ARM_TABLE["BLOCKING_UP"],
                        "dance move with right arm above head"),

        # 13
        TORSO_MONKEY_RIGHT_UP:
        Torso_Position(TORSO_ARM_TABLE["JOHNNY_BRAVO_MONKEY_UP"],
                        TORSO_ARM_TABLE["JOHNNY_BRAVO_MONKEY_DOWN"],
                        "starting johnny bravo's monkey dance"),

        TORSO_MONKEY_LEFT_UP:
        Torso_Position(TORSO_ARM_TABLE["JOHNNY_BRAVO_MONKEY_DOWN"],
                        TORSO_ARM_TABLE["JOHNNY_BRAVO_MONKEY_UP"],
                        "finishing johnny bravo's monkey dance")


        }


