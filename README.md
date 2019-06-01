# race_on
1/10th size Autonomous car working code. 

The autonomous car consists of a servo motor, main motor, raspberry pi and raspberry pi camera for detecting lanes. The algorithm uses OpenCV for image processing. It detects peaks from the image and uses butterworth filter to filter out the peaks. The algorithm also uses PID for controlling the servo based on the peaks detected. 
