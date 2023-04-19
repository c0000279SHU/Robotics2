# Robotics Assignment 2
The puropse of this robot is to be a quadruped robot capiable of moving up and down.
Additinoally here is a link to the video demonstration: https://drive.google.com/file/d/1GprLLe_lFJtb0l0Ae_kWWzTAOXGwnTiy/view?usp=sharing
## FluxUpDown
FluxUpDown is a folder containing the FluxUpDown.ino file written in c++ for the Arduino MKR Wifi 1010. 
I will use the following headers to descibe the function calls and their puropses. All code was written, compiled, and uploaded in Arduino IDE 2.0.4.

### setup()
Initializes the adafruit 16 channel servo controller

### centerHips()
Moves the servos controlling the hips to a centered position so the legs are up and down and not forwards or otherwise skewed.

### midLegs()
Moves the servos controlling the legs so that the legs rest in a neutral position, halfway between a sitting and standing position.


### stand()
Slowly eases the servos controlling the legs to a standing position from the previous position.

### sit()
Slowly eases the servos controlling the legs to a sitting position from the previous position.

### initLegs()
Sets the hip servos to the center position and sets the legs to the neutral or mid position.

### loop()
constantly loops to get the data from the controller and tells the robot how to respond accordingly.

## Uploading to Arduino
I've found that the best way to upload to the Arduino is by utilizing a Mac and ideally Arduino IDE 2.0.4 as this will not cause the Arduino to derecognize itself when entering bootloader mode as there is no com ports to deal with. 

## Once Uploaded
After you've uploaded a sketch to the Arduino, power on the external battery back or power supply to the adafruit servo controller, and place the robot so it is evenly on all four of its legs. 

## Usage Instructions
After uploading, press the joystick forwards for the robot to enter a sitting position and release to enter neutral position. Push the joystick backwards for the robot to stand up and release to enter neutral position.

### references to sources
The only source I had utilized is the library for the Adafruit 16 channel servo controller. https://learn.adafruit.com/16-channel-pwm-servo-driver
