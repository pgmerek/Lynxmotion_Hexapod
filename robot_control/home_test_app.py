import time
from hex_walker_driver import *

def leg_driver_test(leg):
    print("--------------------------leg driver test---------------------")
    leg.set_angle(0, TIP_MOTOR)
    leg.set_angle(45, TIP_MOTOR)
    leg.set_angle(90, TIP_MOTOR)
    leg.set_angle(180, TIP_MOTOR)
    leg.set_angle(270, TIP_MOTOR)
    leg.set_angle(0, MID_MOTOR)
    leg.set_angle(45, MID_MOTOR)
    leg.set_angle(90, MID_MOTOR)
    leg.set_angle(180, MID_MOTOR)
    leg.set_angle(270, MID_MOTOR)
    leg.set_angle(0, ROT_MOTOR)
    leg.set_angle(45, ROT_MOTOR)
    leg.set_angle(90, ROT_MOTOR)
    leg.set_angle(180, ROT_MOTOR)
    leg.set_angle(270, ROT_MOTOR)

def test_leg_positions(leg):
    print("-------------------------------leg position test------------------------------")
    print("***************************************testing neutral")
    leg.set_leg_position(NORMAL_TRI_ROTATION_TABLE["NEUTRAL"])
    leg.print_self()
    print("***************************************testing up")
    up = Leg_Position(90, 180, 90)
    leg.set_leg_position(up)
    leg.print_self()
    print("***************************************testing out")
    out = Leg_Position(120, 45, 90)
    leg.set_leg_position(out)
    leg.print_self()

def change_front_test(hw):
    print("--------------------------------hex walker change front test------------------")
    print("********************************************Should show robot with leg 0,5 up")
    #use do_set_hex_walker_position to avoid updating the self.current_pos variable and therefore skipping all safety checks that cause failures
    if(hw.do_set_hex_walker_position(HEX_WALKER_POSITIONS[FRONT_LEGS_UP]) < 0):
        print("failure")
    hw.print_self()
    print("**********************************************Should show robot with leg 1,2 up")
    hw.do_set_hex_walker_position(HEX_WALKER_POSITIONS[NORMAL_NEUTRAL])
    hw.set_new_front("1-2")
    hw.do_set_hex_walker_position(HEX_WALKER_POSITIONS[FRONT_LEGS_UP])
    hw.print_self()
    print("**********************************************Should show robot with leg 2,3 up")
    hw.do_set_hex_walker_position(HEX_WALKER_POSITIONS[NORMAL_NEUTRAL])
    hw.set_new_front("2-3")
    hw.do_set_hex_walker_position(HEX_WALKER_POSITIONS[FRONT_LEGS_UP])
    hw.print_self()
    
def move_set_test(hw):
    moves = [   NORMAL_NEUTRAL,
                NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                NORMAL_TRI_RIGHT_BACK_LEFT_UP_FORWARD,
                NORMAL_TRI_RIGHT_BACK_LEFT_FORWARD,
                NORMAL_TRI_RIGHT_UP_BACK_LEFT_FORWARD,
                NORMAL_TRI_RIGHT_UP_NEUTRAL_LEFT_NEUTRAL,
                NORMAL_TRI_RIGHT_UP_FORWARD_LEFT_BACK,
                NORMAL_TRI_RIGHT_FORWARD_LEFT_BACK,
                NORMAL_TRI_RIGHT_FORWARD_LEFT_UP_BACK,
                NORMAL_TRI_RIGHT_NEUTRAL_LEFT_UP_NEUTRAL,
                NORMAL_NEUTRAL]
    hw.do_move_set(moves)


lf_leg = Leg(5, 0x41, 0, 1, 2, LEFT)
lm_leg = Leg(4, 0x41, 3, 4, 5, LEFT)
lr_leg = Leg(3, 0x41, 6, 7, 8, LEFT)
rr_leg = Leg(2, 0x41, 9, 10, 11, RIGHT)
rm_leg = Leg(1, 0x41, 12, 13, 14, RIGHT)
rf_leg = Leg(0, 0x40, 0, 1, 2, RIGHT)

hex_walker = Hex_Walker(rf_leg, rm_leg, rr_leg, lr_leg, lm_leg, lf_leg)
move_set_test(hex_walker)
