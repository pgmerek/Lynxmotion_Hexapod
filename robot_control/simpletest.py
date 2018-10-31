# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685(address=0x40)
channel = 6
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)


# code goes here

print('Moving servo on channel "+ str(channel) + ", press Ctrl-C to quit...')
while True:
    # Move servo on channel O between extremes.
    for i in range(300, 1000, 1):
      val =  i
      print("setting to "+str(val))
      #pwm.set_pwm(channel, something, pwm_value)
      pwm.set_pwm(channel, 0, val)
      time.sleep(.1)
