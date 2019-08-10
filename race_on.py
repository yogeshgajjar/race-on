###########################################################################
#    SOURCE CODE FOR MIDDLE LANE DETECTION USING 1/10th AUTONOMOUS CAR 
###########################################################################

# INITIAL DECLARATION OF LIBRARIES 
from picamera.array import PiYUVArray, PiRGBArray
from picamera import PiCamera
from scipy.signal import find_peaks, butter, filtfilt
import time
import numpy as np
from pwm import PWM  #PWM imported from another script. 


# PWM1 is for servo motor which helps in turning of the car. 
# Initialization of PWM and making the front wheel facing front. 
#1500000 gives centre value, 1000000 gives right turn & 2000000 give left turn

pwm1 = PWM(1)
pwm1.export()
pwm1.period = 20000000
pwm1.duty_cycle = 1500000
pwm1.enable = True

# PWM0 is for main motor for moving the car. 
# Initialization of PWM. 
#1500000 gives centre value, 1000000 gives right turn & 2000000 give left turn

pwm0 = PWM(0)
pwm0.export()
pwm0.period = 20000000
pwm0.duty_cycle = 1000000
pwm0.enable = True
time.sleep(5)

def pwmservo(error,t1, pre_error):
    pwm0.duty_cycle=1125000
    Kp = 1500       #setting up the values of Kp, Ki, Kd
    Ki = 700
    Kd = 300
    all_error = 0
    all_error = all_error + error
    output = Kp*error + Ki*all_error*t1 + Kd * (error - pre_error)  #calculating output with PID
    if error > 100:
        if (round(output)) > 500000:
            pwm1.duty_cycle = 1000000  #turns max left
        else:
            pwm1.duty_cycle = 1500000 - (int(round(output))) #1500000 gives centre value, 1000000 gives right turn & 2000000 give left turn
            pwm1.enable = True

    elif error < -100:
        if abs(round(output)) > 500000:
            pwm1.duty_cycle = 2000000  #turns max right

        else:
            pwm1.duty_cycle = 1500000 - (int(round(output))) 
            pwm1.enable = True

    else:
        pwm1.duty_cycle = 1500000  
        pwm1.enable = True


def callmotoroff(): # To make the main motor stop
    pwm0.duty_cycle = 1000000
    pwm0.enable = False
    print("Motor off")

def callservooff(): # To make the servo stop
    pwm1.duty_cycle = 1000000
    pwm1.duty_cycle = 1500000
    pwm1.enable = False
    print("Servo off")


# Camera resolution
res = (640, 480)

# To filter the noise in the image we use a 3rd order Butterworth filter

# Wn = 0.007, the cut-off frequency, acceptable values are from 0 to 1
b, a = butter(3, 0.007)
camera = PiCamera()


camera.sensor_mode = 7
camera.resolution = res
camera.framerate = 120
camera.rotation = 180

# Initialize the buffer and start capturing
rawCapture = PiYUVArray(camera, size=res)
stream = camera.capture_continuous(rawCapture, format="yuv", use_video_port=True)

# Measure the time needed to process 550 images to estimate the FPS
N = 550
k = 0
t = time.time()
pre_xmax, time_elapsed, t1, pre_error = 0
for f in stream:
    # Get the intensity component of the image (a trick to get black and white images)
    I = f.array[:, :, 0]

    # Reset the buffer for the next image
    rawCapture.truncate(0)

    # Select a horizontal line in the middle of the image
    L = I[120, :]

    # Smooth the transitions so we can detect the peaks
    Lf = filtfilt(b, a, L)

    # Find peaks which are higher than 0.5
    p = find_peaks(Lf)
    xmax = np.argmax(Lf)
    ymax = np.max(Lf)
    ymin = np.min(Lf)

    # Identifying error from the peaks
    if (ymax - ymin > 50):
        error = 320 - xmax
        pre_xmax = xmax
        pwmservo(error, t1, pre_error)
    else:
        error = 320 - pre_xmax
        pwmservo(error, t1, pre_error)
    pre_error = error

    time_elapsed = time.time() - t
    t1 = time_elapsed/N


    # Increment the number of processed frames
    k += 1
    if k > N:
        break

print("Elapsed {:0.2f} seconds, estimated FPS {:0.2f}".format(time_elapsed, N / time_elapsed))
callservooff()


# Release resources
stream.close()
rawCapture.close()
camera.close()
callmotoroff()
pwm1.duty_cycle = 1500000
print("All Done")
