# This project is intended to be fully open source. For questions feel free to contact me at girondamatteo@gmail.com

## Mechanical Design:

All SolidWorks CAD files are found in the CAD folder. All 3D printed parts are in the STL subdriectory.

### CAD Assembly:

![alt text here](Pictures/Pendulum_CAD.PNG)

### Bill of Materials:

> •	Raspberry Pi (any version with 20 GPIO pins)

> •	Raspberry Pi HAT for DC Motors (https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi)

> •	12V DC motor (mine has a max speed of 500 rpm and 1A max)

> •	BNO08x sensor (https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/arduino)

> •	9x9x2 in wooden block

> •	Motor attachment to fit with motor shaft (mine had a 6mm shaft https://www.amazon.com/gp/product/B00B887FX8/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1)

> •	Screws for motor attachment (mine were 6x #4-40 x1”)

> •	Keyed 1” Diameter shaft 6” in length

> •	Key (mine was ¼” x ¼”, 1-3/4” long, depends on the keyway of the shaft)

> •	2x Mounted ball bearings for shaft (mounts could theoretically be 3D printed, I bought mine with mounting points)

> •	¼” – 20 thread. 1-1/2” long thumb screw and nut (this is to clamp the motor to the top of the arm)

> •	Self tapping screws for the bearing mounts and side supports (I ended up going with a ½” size, 1-1/2” long; definitely an overkill and not as easy to drive into wood. Make sure you have the appropriate drive bits and socket adapters)

> •	Steel (or metal with similar density) ring (https://www.ebay.com/itm/203450660510)

> •	Access to a 3D printer, soldering iron, appropriate DC power supplies (separate for Pi and motor), connectors, drill, foam, tape.  

### Front View of Pendulum:
![alt text here](Pictures/Pendulum_Front.jpg)

### Side View of Pendulum:
![alt text here](Pictures/Pendulum_Side.jpg)

The mechanical design was inspred by https://www.researchgate.net/publication/331271615_Design_Modeling_and_Control_of_a_Reaction_Wheel_Balanced_Inverted_Pendulum and its authors. 

## Controller

The idea is to send a PWM signal to the motor to rotate the reaction wheel against the motion of the pendulum arm. Gains are tuned to adjust the needed voltage input based on the state vector of the arm. States are updated from the IMU in the feedback loop. The control input becomes saturated at 12V and -12V.

A simulation of the equations of motion and state vector algebra is found in simulation.py. The simulation allows you to change starting conditions and gains, and outputs control effort and states. 

## Notes

I am using UART communication over I2C because the BNO08x series has clock stretching issues when interfacing with the default I2C bus of the pi. This can be worked around by using different SCL and SDA pins on the GPIO that use hardware clock stretching instead of software clock stretching (see https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/issues/7). I found that it's just simpler to use UART for this application, as the motor driver is already using the pi's default I2C bus.
