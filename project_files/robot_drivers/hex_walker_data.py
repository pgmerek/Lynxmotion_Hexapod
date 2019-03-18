from leg_data import *


class Hex_Walker_Position(object):  # Class name should be camelcase but I'll let it go
    """
    Object to store the positions of all legs for a desired stance. Also has a list of all save
    moves that the hexapod can make from this stance.
    """
    def __init__(self, rf_pos, rm_pos, rr_pos, lr_pos, lm_pos, lf_pos, safe_move_list, description):
        """
        :param rf_pos: Right front leg position
        :param rm_pos: Right mid leg position
        :param rr_pos: Right rear leg position
        :param lr_pos: Left rear leg position
        :param lm_pos: Left mid leg position
        :param lf_pos: Left front leg position
        :param safe_move_list: List of approved moves that won't damage the robot
        :param description: Short description of the current stance
        """
        self.rf_pos = rf_pos
        self.rm_pos = rm_pos
        self.rr_pos = rr_pos
        self.lr_pos = lr_pos
        self.lm_pos = lm_pos
        self.lf_pos = lf_pos
        self.safe_moves = safe_move_list
        self.description = description

    def __str__(self):
        """
        Simple function to assemble a console message when the hex walker object is instantiated
        :return: String with the positions of each leg in clockwise order
        """
        start_str = "--------------------------hex_walker position is------------------\n"
        rf_str = "rf: " + str(self.rf_pos) + "\n"
        rm_str = "rm: " + str(self.rm_pos) + "\n"
        rr_str = "rr: " + str(self.rr_pos) + "\n"
        lr_str = "lr: " + str(self.lr_pos) + "\n"
        lm_str = "lm: " + str(self.lm_pos) + "\n"
        lf_str = "lf: " + str(self.lf_pos) + "\n"
        return start_str + rf_str + rm_str + rr_str + lr_str + lm_str + lf_str


# NOTE: I have left in repeated steps and simply commented them out.
# It helps for continuity and error checking since you can see the entire process

# Enumerated list of all possible hex_walker positions
# possible hex_walker positions during a tripod "walk" cycle
NORMAL_NEUTRAL = 1
NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL = 2
NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD = 3
NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD = 4
NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD = 5
NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL = 6
NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK = 7
NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK = 8
NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK = 9
# NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "rotate" cycle
# NORMAL_NEUTRAL
# NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT = 10
NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT = 11
NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT = 12
# NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT = 13
NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT = 14
NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT = 15
# NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "walk" cycle
CROUCH_NEUTRAL = 16
CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL = 17
CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD = 18
CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD = 19
CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD = 20
CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL = 21
CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK = 22
CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK = 23
CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK = 24
# CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "rotate" cycle
# CROUCH_NEUTRAL
# CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT = 25
CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT = 26
CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT = 27
# CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT = 28
CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT = 29
CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT = 30
# CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "walk" cycle
TALL_NEUTRAL = 31
TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL = 32
TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD = 33
TALL_TRI_RIGHT_BACK_LEFT_FORWARD = 34
TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD = 35
TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL = 36
TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK = 37
TALL_TRI_RIGHT_FORWARD_LEFT_BACK = 38
TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK = 39
# TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "rotate" cycle
# TALL_NEUTRAL
# TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT = 40
TALL_TRI_RIGHT_RIGHT_LEFT_LEFT = 41
TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT = 42
# TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT = 43
TALL_TRI_RIGHT_LEFT_LEFT_RIGHT = 44
TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT = 45
# TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL

# possible hex_walker positions during a tripod "side walk" cycle
# "front" doesn't refer to the label on the robot. The front is just the side that the robot is moving towards.

# TALL_NEUTRAL = 46
TALL_TRI_FRONT_CENTER_UP_OUT_BACK_NEUTRAL = 46
TALL_TRI_FRONT_CENTER_OUT_BACK_UP_NEUTRAL = 47
TALL_TRI_FRONT_BACKWARDS_BACK_UP_NEUTRAL= 48
TALL_TRI_FRONT_BACKWARDS_BACK_NEUTRAL= 49
TALL_TRI_FRONT_UP_NEUTRAL_BACK_NEUTRAL = 50 
TALL_TRI_FRONT_UP_NEUTRAL_BACK_BACKWARDS = 51
TALL_TRI_FRONT_NEUTRAL_BACK_BACKWARDS = 52
TALL_TRI_FRONT_NEUTRAL_BACK_UP_NEUTRAL = 53

# bounce position
TALL_TRI_BOUNCE_DOWN = 54

# Fine rotations 
TALL_TRI_FINE_RIGHT_RIGHT_LEFT_UP_LEFT = 55
TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT = 56
TALL_TRI_FINE_RIGHT_UP_RIGHT_LEFT_LEFT = 57
# TALL_TRI_FINE_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
TALL_TRI_FINE_RIGHT_UP_LEFT_LEFT_RIGHT = 58
TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT = 59
TALL_TRI_FINE_RIGHT_LEFT_LEFT_UP_RIGHT = 60
# testing positions
FRONT_LEGS_UP = 1001

# these are all defines as hex_walker_position(rf, rm, rr, lr, lm, lf)
HEX_WALKER_POSITIONS = {
        # Normal (standard height) walking positions the order that they need to execute
        # 1
        NORMAL_NEUTRAL:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [NORMAL_NEUTRAL, CROUCH_NEUTRAL,
                             NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             TALL_NEUTRAL
                             ],
                            "normal neutral position",
                            ),
        # 2
        NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [NORMAL_NEUTRAL,
                             NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                             NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT
                             ],
                            "right is neutral, left is up",
                            ),
        # 3
        NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_UP_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_OUT"],
                            [NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD
                             ],
                            "right back, left up",
                            ),
        # 4
        NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            [NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD,
                             NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD
                             ],
                            "right is back, left is forward",
                            ),
        # 5
        NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_UP_LEFT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            [NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD
                             ],
                            "right is up, left is forward",
                            ),
        # 6
        NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            NORMAL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             NORMAL_NEUTRAL,
                             NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                             NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT
                             ],
                            "right is up, left is neutral",
                            ),
        # 7
        NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_UP_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            [NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is up, left is back",
                            ),
        # 8
        NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            [NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK,
                             NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK
                             ],
                            "right is forward, left is back",
                            ),
        # 9
        NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK:
        Hex_Walker_Position(NORMAL_TRI_MOVEMENT_TABLE["CORN_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_UP_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_IN"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_OUT"],
                            NORMAL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            NORMAL_TRI_MOVEMENT_TABLE["CORN_UP_IN"],
                            [NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is forward, left is up",
                            ),
        
        # Normal rotation movements
        # 10
        NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            [NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                            NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                            NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT
                            ],
                            "right is right, left is up",
                            ),
        # 11
        NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            [NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            NORMAL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT
                            ],
                            "right is right, left is left",
                            ),
        # 12
        NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            [NORMAL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                            NORMAL_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            ],
                            "right is up, left is left",
                            ),
        # 13
        NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            [NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
                            NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
                            ],
                            "right is up, left is right",
                            ),
        # 14
        NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["RIGHT"],
                            [NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            NORMAL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT
                            ],
                            "Right is left, left is right",
                            ),
        # 15
        NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT:
        Hex_Walker_Position(NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            NORMAL_TRI_ROTATION_TABLE["LEFT"],
                            NORMAL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            [NORMAL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            NORMAL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
                            ],
                            "right is left, left is up",
                            ),
        # Crouch (low height) walking positions the order that they need to execute
        # 16
        CROUCH_NEUTRAL:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [CROUCH_NEUTRAL, NORMAL_NEUTRAL,
                             CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
                             ],
                            "crouch neutral position",
                            ),

        # 17
        CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                             CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                             CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             CROUCH_NEUTRAL
                             ],
                            "right is neutral, left is up",
                            ),
        # 18
        CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_UP_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_UP_OUT_RIGHT"],
                            [CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD
                             ],
                            "right neutral, left up",
                            ),
        # 19
        CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            [CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD,
                             CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             CROUCH_TRI_RIGHT_BACK_LEFT_UP_FORWARD
                             ],
                            "right is neutral, left is forward",
                            ),
        # 20
        CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_UP_OUT_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            [CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             CROUCH_TRI_RIGHT_BACK_LEFT_FORWARD,
                             ],
                            "right is up, left is forward",
                            ),
        # 21
        CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
                             CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                             CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             CROUCH_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             CROUCH_NEUTRAL
                             ],
                            "right is up, left is neutral",
                            ),
        # 22
        CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["CORN_UP_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_UP_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is up, left is neutral",
                            ),
        # 23
        CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK,
                             CROUCH_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK
                             ],
                            "right is forward, left is neutral",
                            ),
        # 24
        CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK:
        Hex_Walker_Position(CROUCH_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            CROUCH_TRI_MOVEMENT_TABLE["CORN_UP_OUT_LEFT"],
                            CROUCH_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            CROUCH_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [CROUCH_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             CROUCH_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is forward, left is up",
                            ),
        
        # crouch rotation movements
        # 25
        CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            [CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                            CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                            CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT
                            ],
                            "right is right, left is up",
                            ),
        # 26
        CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            [CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            CROUCH_TRI_RIGHT_RIGHT_LEFT_UP_LEFT
                            ],
                            "right is right, left is left",
                            ),
        # 27
        CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            [CROUCH_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                            CROUCH_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            ],
                            "right is up, left is left",
                            ),
        # 28
        CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            [CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
                            CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            CROUCH_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
                            ],
                            "right is up, left is right",
                            ),
        # 29
        CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["RIGHT"],
                            [CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            CROUCH_TRI_RIGHT_UP_LEFT_LEFT_RIGHT
                            ],
                            "Right is left, left is right",
                            ),
        # 30
        CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT:
        Hex_Walker_Position(CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            CROUCH_TRI_ROTATION_TABLE["LEFT"],
                            CROUCH_TRI_ROTATION_TABLE["UP_RIGHT"],
                            [CROUCH_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            CROUCH_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            CROUCH_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
                            ],
                            "right is left, left is up",
                            ),

        # Tall (tall height) walking positions the order that they need to execute
        # 31
        TALL_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_NEUTRAL, NORMAL_NEUTRAL,
                             TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             TALL_TRI_FRONT_CENTER_UP_OUT_BACK_NEUTRAL,
                             TALL_TRI_BOUNCE_DOWN
                             ],
                            "tall neutral position",
                            ),

        # 32
        TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                             TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                             TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             TALL_NEUTRAL
                             ],
                            "right is neutral, left is up",
                            ),
        # 33
        TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_UP_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_UP_OUT_RIGHT"],
                            [TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                             TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             TALL_TRI_RIGHT_BACK_LEFT_FORWARD
                             ],
                            "right neutral, left up",
                            ),
        # 34
        TALL_TRI_RIGHT_BACK_LEFT_FORWARD:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            [TALL_TRI_RIGHT_BACK_LEFT_FORWARD,
                             TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             TALL_TRI_RIGHT_BACK_LEFT_UP_FORWARD
                             ],
                            "right is neutral, left is forward",
                            ),
        # 35
        TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_UP_OUT_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_RIGHT"],
                            [TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             TALL_TRI_RIGHT_BACK_LEFT_FORWARD,
                             ],
                            "right is up, left is forward",
                            ),
        # 36
        TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
                             TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                             TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             TALL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                             TALL_TRI_FINE_RIGHT_RIGHT_LEFT_UP_LEFT,
                             TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT,
                             TALL_TRI_FINE_RIGHT_UP_RIGHT_LEFT_LEFT,
                             TALL_TRI_FINE_RIGHT_UP_LEFT_LEFT_RIGHT,
                             TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT,
                             TALL_TRI_FINE_RIGHT_LEFT_LEFT_UP_RIGHT,
                             TALL_NEUTRAL
                             ],
                            "right is up, left is neutral",
                            ),
        # 37
        TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["CORN_UP_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_UP_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                             TALL_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is up, left is neutral",
                            ),
        # 38
        TALL_TRI_RIGHT_FORWARD_LEFT_BACK:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_RIGHT_FORWARD_LEFT_BACK,
                             TALL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                             TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK
                             ],
                            "right is forward, left is neutral",
                            ),
        # 39
        TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK:
        Hex_Walker_Position(TALL_TRI_MOVEMENT_TABLE["CORN_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_MOVEMENT_TABLE["CORN_UP_OUT_LEFT"],
                            TALL_TRI_MOVEMENT_TABLE["SIDE_RIGHT"],
                            TALL_TRI_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [TALL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                             TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                             TALL_TRI_RIGHT_FORWARD_LEFT_BACK
                             ],
                            "right is forward, left is up",
                            ),
        
        # crouch rotation movements
        # 40
        TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            [TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT,
                            TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                            TALL_TRI_RIGHT_RIGHT_LEFT_LEFT
                            ],
                            "right is right, left is up",
                            ),
        # 41
        TALL_TRI_RIGHT_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            [TALL_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            TALL_TRI_RIGHT_RIGHT_LEFT_UP_LEFT
                            ],
                            "right is right, left is left",
                            ),
        # 42
        TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            [TALL_TRI_RIGHT_UP_RIGHT_LEFT_LEFT,
                            TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                            TALL_TRI_RIGHT_RIGHT_LEFT_LEFT,
                            ],
                            "right is up, left is left",
                            ),
        # 43
        TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            [TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
                            ],
                            "right is up, left is right",
                            ),
        # 44
        TALL_TRI_RIGHT_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["RIGHT"],
                            [TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            TALL_TRI_RIGHT_UP_LEFT_LEFT_RIGHT
                            ],
                            "Right is left, left is right",
                            ),
        # 45
        TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT:
        Hex_Walker_Position(TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_ROTATION_TABLE["LEFT"],
                            TALL_TRI_ROTATION_TABLE["UP_RIGHT"],
                            [TALL_TRI_RIGHT_LEFT_LEFT_UP_RIGHT,
                            TALL_TRI_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
                            ],
                            "right is left, left is up",
                            ),

        # 46
        TALL_TRI_FRONT_CENTER_UP_OUT_BACK_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["CENTER_UP_OUT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_FRONT_CENTER_OUT_BACK_UP_NEUTRAL
                            ],
                            "front leg up-out, all others neutral",
                            ),


        # 47
        TALL_TRI_FRONT_CENTER_OUT_BACK_UP_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["CENTER_OUT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [TALL_TRI_FRONT_BACKWARDS_BACK_UP_NEUTRAL
                            ],
                            "front leg out, all others neutral",
                            ),

        # 48
        TALL_TRI_FRONT_BACKWARDS_BACK_UP_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_RIGHT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_LEFT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [TALL_TRI_FRONT_BACKWARDS_BACK_NEUTRAL
                            ],
                            "front legs back, all others up neutral",
                            ),

        # 49
        TALL_TRI_FRONT_BACKWARDS_BACK_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_RIGHT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_LEFT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_FRONT_UP_NEUTRAL_BACK_NEUTRAL
                            ],
                            "front legs back, all others neutral",
                            ),
        # 50
        TALL_TRI_FRONT_UP_NEUTRAL_BACK_NEUTRAL: 
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            [TALL_TRI_FRONT_UP_NEUTRAL_BACK_BACKWARDS
                            ],
                            "front legs up neutral, all others neutral",
                            ),
        
        # 51
        TALL_TRI_FRONT_UP_NEUTRAL_BACK_BACKWARDS:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_RIGHT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["CENTER_OUT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_LEFT"],
                            [TALL_TRI_FRONT_NEUTRAL_BACK_BACKWARDS
                            ],
                            "front legs up neutral, all others back",
                            ),

        # 52
        TALL_TRI_FRONT_NEUTRAL_BACK_BACKWARDS:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_RIGHT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["CENTER_OUT"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["SIDE_OUT_LEFT"],
                            [TALL_TRI_FRONT_NEUTRAL_BACK_UP_NEUTRAL
                            ],
                            "front legs neutral, all others back",
                            ),
        # 53
        TALL_TRI_FRONT_NEUTRAL_BACK_UP_NEUTRAL:
        Hex_Walker_Position(TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["NEUTRAL"],
                            TALL_TRI_SIDE_MOVEMENT_TABLE["UP_NEUTRAL"],
                            [TALL_NEUTRAL
                            ],
                            "front legs neutral, all others up neutral",
                            ),

        # 54
        TALL_TRI_BOUNCE_DOWN:
        Hex_Walker_Position(MISC_TABLE["BOUNCE"],
                            MISC_TABLE["BOUNCE"],
                            MISC_TABLE["BOUNCE"],
                            MISC_TABLE["BOUNCE"],
                            MISC_TABLE["BOUNCE"],
                            MISC_TABLE["BOUNCE"],
                            [TALL_NEUTRAL
                            ],
                            "crouched down from tall height",
                            ),

        # Fine rotations
        # 55
        TALL_TRI_FINE_RIGHT_RIGHT_LEFT_UP_LEFT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            [TALL_TRI_FINE_RIGHT_RIGHT_LEFT_UP_LEFT,
                            TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                            TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT
                            ],
                            "right is right, left is up",
                            ),
        # 56
        TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            [TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT,
                            TALL_TRI_FINE_RIGHT_UP_RIGHT_LEFT_LEFT,
                            TALL_TRI_FINE_RIGHT_RIGHT_LEFT_UP_LEFT
                            ],
                            "right is right, left is left",
                            ),
        # 57
        TALL_TRI_FINE_RIGHT_UP_RIGHT_LEFT_LEFT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            [TALL_TRI_FINE_RIGHT_UP_RIGHT_LEFT_LEFT,
                            TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                            TALL_TRI_FINE_RIGHT_RIGHT_LEFT_LEFT,
                            ],
                            "right is up, left is left",
                            ),
        # 58
        TALL_TRI_FINE_RIGHT_UP_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            [TALL_TRI_FINE_RIGHT_UP_LEFT_LEFT_RIGHT,
                            TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL
                            ],
                            "right is up, left is right",
                            ),
        # 59
        TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["RIGHT"],
                            [TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_FINE_RIGHT_LEFT_LEFT_UP_RIGHT,
                            TALL_TRI_FINE_RIGHT_UP_LEFT_LEFT_RIGHT
                            ],
                            "Right is left, left is right",
                            ),
        # 60
        TALL_TRI_FINE_RIGHT_LEFT_LEFT_UP_RIGHT:
        Hex_Walker_Position(TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            TALL_TRI_FINE_ROTATION_TABLE["LEFT"],
                            TALL_TRI_FINE_ROTATION_TABLE["UP_RIGHT"],
                            [TALL_TRI_FINE_RIGHT_LEFT_LEFT_UP_RIGHT,
                            TALL_TRI_FINE_RIGHT_LEFT_LEFT_RIGHT,
                            TALL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL
                            ],
                            "right is left, left is up",
                            ),
        # past here are just positions that are used for testing.
        # past here are just positions that are used for testing.
        # They can only be reached by __set_hex_walker_position direct calls
        FRONT_LEGS_UP:
        Hex_Walker_Position(Leg_Position(180, 180, 90),
                            NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                            NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                            NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                            NORMAL_TRI_ROTATION_TABLE["NEUTRAL"],
                            Leg_Position(180, 180, 90),
                            [],
                            "front two legs are raised",
                            )
        }



