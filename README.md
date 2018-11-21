# Fall 2018 - ECE478/578 Robotics 1
**Intelligent Robotics Lab**
 **Intelligent Robotics 1 Course Project**

**Portland State University**

# Lynxmotion Hexapod
Hexapod with 25 degrees of freedom. Driven by Raspberry Pi and Adafruit PWM hats. Drivers for motion written in Python 3.
# Project Status
Currently, the hexapod base can walk using two different gaits, turn in place, and perform some dance moves.

The torso has a plethora of different moves including waving, flexing, hugging, and imitating King Kong.

The torso and hexapod base can be controlled simultaneously using threading in Python. 
The hexapod can also behave depending on the color and size an object it sees. For instance, a small yellow object will cause the hexapod to walk towards the object.

The next stage of this project is to integrate vision, speech recognition, speech synthesis, and motion using Robot Operating System (ROS).
The hexapod needs to be able to interact with the VikingBot for a theatrical performance. Finally, the hexapod must be able to carry out a small conversation with a humaFinally, the hexapod must be able to carry out a small conversation with a human.
# Project Goals
Construct a stable hardware platform from which we can develop drivers for motion, vision, speech recognition, and speech synthesis.
Attach a torso and bobble head for more advanced theatrical behavior and comedic value.

Provide a data-driven, object-orientated motion driver that can be easily added to and maintained. Reduce complexity in code whenever possible and write functions to be generic and robust.

# List of Hardware and Software (Tools, Libraries, etc)
Software:
* Python 3
* Robot Operation System (ROS) Kinetic
* Raspian OS
* OpenCV 3
* AdaFruit PCA9685 Python Library

Hardware:
* Raspberry Pi 3
* PiCam
* Adafruit Servo/PWM Hat (x2)

Future:
* Hitec HS-645MG servos
# Other Notes
This was a new robot with no prior code or documentation.
# Project Team
* Charles Stoll (Integration and Motion)
* Emma Smith (Vision and Hardware)
* Ammar Khan (Speech)
* Patrick Gmerek (Hardware and Repo Master)
