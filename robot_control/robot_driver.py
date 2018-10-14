from __future__ import division
import time
import Adafruit_PCA9685


#all constants related to the robot legs
L_TIP_MOTOR_OUT = 600
L_TIP_MOTOR_IN = 300
L_TIP_MOTOR_MID = (TIP_MOTOR_OUT + TIP_MOTOR_IN)/2
L_TIP_MOTOR_OUT_ANGLE = 180
L_TIP_MOTOR_IN_ANGLE = 45
L_MID_MOTOR_UP = 120
L_MID_MOTOR_DOWN = 450
L_MID_MOTOR_MID = (MID_MOTOR_UP + MID_MOTOR_DOWN)/2
L_MID_MOTOR_UP_ANGLE = 180
L_MID_MOTOR_DOWN_ANGLE = 45
L_ROT_MOTOR_FOR = 140
L_ROT_MOTOR_BACK = 590
L_ROT_MOTOR_FOR_ANGLE = 180
L_ROT_MOTOR_BACK_ANGLE = 0
L_ROT_MOTOR_CENT = (ROT_MOTOR_FOR + ROT_MOTOR_BACK)/2

TIP_MOTOR = 1
MID_MOTOR = 2
ROT_MOTOR = 3

LEFT = 1
RIGHT = 2

#error codes and coding constants
SUCCESS = 0
INV_PARAM = -1

#helper functions
#returns slope given two points
def slope(x1, y1, x2, y2):
  return (y2-y1)/(x2-x1)

def intercept(x2, y2, slope):
  return y2 - slope * x2

def linear_map(x1, y1, x2, y2, x_in_val):
  m = slope(x1, y1, x2, y2)
  b = intercept(x2, y2, m)
  return x_in_val * m + b

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse = int(pulse)
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

#returns the angle on success. INV_PARAM otherwise
def angle_to_pwm(angle, motor):
  if(motor == TIP_MOTOR):
    return linear_map(TIP_MOTOR_OUT_ANGLE, TIP_MOTOR_OUT, TIP_MOTOR_IN_ANGLE, TIP_MOTOR_IN, angle)
  elif(motor == MID_MOTOR):
    return linear_map(MID_MOTOR_UP_ANGLE, MID_MOTOR_UP, MID_MOTOR_DOWN_ANGLE, MID_MOTOR_DOWN, angle)
  elif(motor == ROT_MOTOR):
    return linear_map(ROT_MOTOR_FOR_ANGLE, ROT_MOTOR_FOR, ROT_MOTOR_BACK_ANGLE, ROT_MOTOR_BACK, angle)
  else:
    return INV_PARAM

#a leg object should be able to associate itself with specific channels
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
    self.tip_motor = TIP_MOTOR_IN
    self.mid_motor = MID_MOTOR_UP
    self.rot_motor = ROT_MOTOR_CENT

  def set_angle(self, angle, motor):
    #sanity checking for each motor
    if(motor == TIP_MOTOR):
      #get pwm val
      pwm_val = int(angle_to_pwm(angle, motor))
      
      #safety check
      if(pwm_val < TIP_MOTOR_IN):
        pwm_val = TIP_MOTOR_IN
      
      elif(pwm_val > TIP_MOTOR_OUT):
        pwm_val = TIP_MOTOR_OUT
      
      #do the write out
      self.pwm.set_pwm(self.tip_channel, 0, pwm_val)
    
    
    elif(motor == MID_MOTOR):
      #get pwm val
      pwm_val = int(angle_to_pwm(angle, motor))
      
      #safety check
      if(pwm_val > MID_MOTOR_DOWN):
        pwm_val = MID_MOTOR_DOWN
      
      elif(pwm_val < MID_MOTOR_UP):
        pwm_val = MID_MOTOR_UP
      
      #do the write out
      self.pwm.set_pwm(self.mid_channel, 0, pwm_val)


    elif(motor == ROT_MOTOR):
      #get pwm val
      pwm_val = int(angle_to_pwm(angle, motor))
      
      #safety check
      if(pwm_val > ROT_MOTOR_BACK):
        pwm_val = ROT_MOTOR_BACK
      
      elif(pwm_val < ROT_MOTOR_FOR):
        pwm_val = ROT_MOTOR_FOR
      
      #do the write out
      self.pwm.set_pwm(self.rot_channel, 0, pwm_val)
    
