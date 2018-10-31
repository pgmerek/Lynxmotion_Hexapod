from leg_data import *
from hex_walker_data import *

TORSO_ARM_TABLE = {
        "NEUTRAL"           : Leg_Position(90, 90, 90),
        "ON_HIP"        : Leg_Position(90, 135, 90),
        "UP"             : Leg_Position(95, 90, 120),
        "STRAIGHT_OUT"   : Leg_Position(90,90,90),
        "STRAIGHT_FOR"      :Leg_Position(90,90,90),

        "WAVE_UP"          : Leg_Position(95, 135, 120),
        "WAVE_DOWN"         : Leg_Position(90,90,90),
        
        "HAND_SHAKE_UP"     :Leg_Position(90,90,90),
        "HAND_SHAKE_DOWN"   : Leg_Position(90,90,90),
        
        "JOHNNY_BRAVO_MONKEY_UP" : Leg_Position(90,90,90),
        "JOHNNY_BRAVO_MONKEY_UP"  : Leg_Position(90,90,90),

        "BLOCKING_UP"              : Leg_Position(95, 90, 60),
        "BLOCKING_FRONT"           : Leg_Position(95, 135, 60)

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
        Torso_Position(TORSO_POSITIONS("NEUTRAL"),
                        TORSO_POSITIONS("NEUTRAL"),
                        "torso is in the neutral position"),
        # 2
        TORSO_JACK_DOWN:
        Torso_Position(TORSO_POSITIONS("WAVE_DOWN"),
                        TORSO_POSITIONS("WAVE_DOWN"),
                        "jumping jacks (down pos)"),
        
        # 3
        TORSO_JACK_UP:
        Torso_Position(TORSO_POSITIONS("WAVE_UP"),
                        TORSO_POSITIONS("WAVE_UP"),
                        "jumping jacks (up pos)"),
        # 4
        TORSO_WAVE_DOWN:
        Torso_Position(TORSO_POSITIONS("WAVE_DOWN"),
                        TORSO_POSITIONS("NEUTRAL"),
                        "waving with the right hand (down pos)"),

        # 5
        TORSO_WAVE_UP:
        Torso_Position(TORSO_POSITIONS("WAVE_UP"),
                        TORSO_POSITIONS("NEUTRAL"),
                        "waving with the right hand (up pos)"),
        # 6
        TORSO_SHAKE_DOWN:
        Torso_Position(TORSO_POSITIONS("HAND_SHAKE_DOWN"),
                        TORSO_POSITIONS("NEUTRAL"),
                        "handshaking with the right hand (down pos)"),
        
        # 7
        TORSO_SHAKE_UP:
        Torso_Position(TORSO_POSITIONS("HAND_SHAKE_UP"),
                        TORSO_POSITIONS("NEUTRAL"),
                        "handshaking with the right hand (up pos)"),
        # 8
        TORSO_DANCE_FRONT_LEFT_OUT:
        Torso_Position(TORSO_POSITIONS("BLOCKING_FRONT"),
                        TORSO_POSITIONS("STRAIGHT_OUT"),
                        "dance move with left arm out"),
        # 9
        TORSO_DANCE_FRONT_RIGHT_OUT:
        Torso_Position(TORSO_POSITIONS("STRAIGHT_OUT"),
                        TORSO_POSITIONS("BLOCKING_FRONT"),
                        "dance move with right arm out"),
        # 10
        TORSO_DANCE_ABOVE_LEFT_UP:
        Torso_Position(TORSO_POSITIONS("BLOCKING_UP"),
                        TORSO_POSITIONS("WAVE_DOWN"),
                        "dance move with left arm above head"),
        
        # 11
        TORSO_DANCE_ABOVE_RIGHT_UP:
        Torso_Position(TORSO_POSITIONS("WAVE_DOWN"),
                        TORSO_POSITIONS("BLOCKING_UP"),
                        "dance move with right arm above head"),

        # 13
        TORSO_MONKEY_RIGHT_UP:
        Torso_Position(TORSO_POSITIONS("JOHNNY_BRAVO_MONKEY_UP"),
                        TORSO_POSITIONS("JOHNNY_BRAVO_MONKEY_DOWN"),
                        "starting johnny bravo's monkey dance"),

        TORSO_MONKEY_LEFT_UP:
        Torso_Position(TORSO_POSITIONS("JOHNNY_BRAVO_MONKEY_DOWN"),
                        TORSO_POSITIONS("JOHNNY_BRAVO_MONKEY_UP"),
                        "finishing johnny bravo's monkey dance")


        }


