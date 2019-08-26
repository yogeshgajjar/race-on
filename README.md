# race_on

1/10th size autonomous car with lane detection 

## Overview 

The autonomous car consists of a servo motor, main motor, raspberry pi and raspberry pi camera for detecting lanes. The algorithm uses OpenCV for image processing. It detects peaks from the image and uses butterworth filter to filter out the noise. The algorithm also uses PID for controlling the servo based on the peaks detected. 

This code is for tracking the lane on which the car is moving such that the PID control will keep the car back on track. 

## Stack

- OpenCV for image processing
- Python
- PID for control

## PWM source code

To control the servo and the motor from python, you will have to use the pmwpy module from here https://github.com/scottellis/pwmpy

Refer README from the above mentioned repository for using PWM for motor and servo. 