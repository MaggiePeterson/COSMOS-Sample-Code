# This program illustrates how to capture a images as part of a video stream
# and how to do extract pixels of a specific color
# It includes a slider to adjust the color that is being filtered
# It uses openCV
import cv2
import picamera
import picamera.array                                   # This needs to be imported explicitly
import time
import numpy as np
import RPi.GPIO as GPIO
import skimage
from skimage.measure import label, regionprops
import math
import matplotlib.pyplot as plt

def nothing(x):
    pass

GPIO.setmode(GPIO.BCM)
 
# set GPIO Pins
GPIO_Apwm = 6
GPIO_Ain1 = 13
GPIO_Ain2 = 19

GPIO_Bin1 = 16
GPIO_Bin2 = 20
GPIO_Bpwm = 21

# Set GPIO direction (IN / OUT)
GPIO.setup(GPIO_Ain1, GPIO.OUT)
GPIO.setup(GPIO_Ain2, GPIO.OUT)
GPIO.setup(GPIO_Apwm, GPIO.OUT)
GPIO.setup(GPIO_Bin1, GPIO.OUT)
GPIO.setup(GPIO_Bin2, GPIO.OUT)
GPIO.setup(GPIO_Bpwm, GPIO.OUT)

# Both motors are stopped 
GPIO.output(GPIO_Ain1, False)
GPIO.output(GPIO_Ain2, False)
GPIO.output(GPIO_Bin1, False)
GPIO.output(GPIO_Bin2, False)

# Set PWM parameters
pwm_frequency = 50

# Create the PWM instances
pwmA = GPIO.PWM(GPIO_Apwm, pwm_frequency)
pwmB = GPIO.PWM(GPIO_Bpwm, pwm_frequency)

# Set the duty cycle (between 0 and 100)
# The duty cycle determines the speed of the wheels
pwmA.start(100)
pwmB.start(100)

# Create a window for later use by the track bar
'''cv2.namedWindow('result1')'''
'''h1, s1, v1 = 0, 125, 40'''
'''img_low = np.zeros((15,512,3),np.uint8)'''

'''cv2.namedWindow('result2')'''
'''h2, s2, v2 = 255, 255, '''
'''img_high = np.zeros((15,512,3),np.uint8)'''

# Define the range colors to filter; these numbers represent HSV
lowerColorThresholdO = np.array([0,115,110])
upperColorThresholdO = np.array([13,255,255])
#upperColorThreshold = np.array([255, 255, 255])
lowerColorThresholdP = np.array([89,91,72])
upperColorThresholdP = np.array([182,182,255])
# Create a track bar
'''cv2.createTrackbar('h1','result1', 0, 255, nothing)
cv2.createTrackbar('s1','result1', 0, 255, nothing)
cv2.createTrackbar('v1','result1', 0, 255, nothing)

cv2.createTrackbar('h2','result2', 0, 255, nothing)
cv2.createTrackbar('s2','result2', 0, 255, nothing)
cv2.createTrackbar('v2','result2', 0, 255, nothing)'''


# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))

# allow the camera to warm up
time.sleep(0.1)




if __name__ == '__main__':
    try:

        # Continuously capture frames from the camera
        # Note that the format is BGR instead of RGB because we want to use openCV later on and it only supports BGR
        for frame in camera.capture_continuous(rawframe, format = "bgr", use_video_port = True):

            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)

            
            # Create a numpy array representing the image
            image = frame.array     

            # Convert for BGR to HSV color space, using openCV
            # The reason is that it is easier to extract colors in the HSV space
            # Note: the fact that we are using openCV is why the format for the camera.capture was chosen to be BGR
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Get info from trackbar and appy to the result
            '''cv2.imshow('result1', img_low)
            h1 =  cv2.getTrackbarPos('h1','result1')
            s1 =  cv2.getTrackbarPos('s1','result1')
            v1 =  cv2.getTrackbarPos('v1','result1')
            img_low[:] = [h1,s1,v1]
            
            cv2.imshow('result2', img_high)
            h2 =  cv2.getTrackbarPos('h2','result2')
            s2 =  cv2.getTrackbarPos('s2','result2')
            v2 =  cv2.getTrackbarPos('v2','result2')
            img_high[:] = [h2,s2,v2]'''
            

            # Threshold the HWV image to get only colors in a range
            # The colors in range are set to white (255), while the colors not in range are set to black (0)
            mask1 = cv2.inRange(image_hsv, lowerColorThresholdO, upperColorThresholdO)
            '''mask2 = cv2.inRange(image_hsv, lowerColorThresholdP, upperColorThresholdP)'''
            
            #print("number of lit of pixels: {}\n".format(np.sum(mask1>0)))
            pixelnum1 = np.sum(mask1>0)
            '''pixelnum2 = np.sum(mask2>0)'''
            # Bitwise AND of the mask and the original image
            '''image_masked = cv2.bitwise_and(image, image, mask = mask)'''
            
            label_m1 = label(mask1)
            regions = regionprops(label_m1)

            fig, ax = plt.subplots()
            ax.imshow(image, cmap=plt.cm.gray)

            for ind, props in enumerate(regions):
                y0, x0 = props.centroid
                orientation = props.orientation
                x1 = x0 + math.cos(orientation) * 0.5 * props.major_axis_length
                y1 = y0 - math.sin(orientation) * 0.5 * props.major_axis_length
                x2 = x0 - math.sin(orientation) * 0.5 * props.minor_axis_length
                y2 = y0 - math.cos(orientation) * 0.5 * props.minor_axis_length
                '''print('Region {}, angle {}'.format(ind,math.fabs(orientation)))'''
                tomato=math.fabs(orientation)
                
            
            
       
            if pixelnum1 < 5000:
                GPIO.output(GPIO_Ain1, True)
                GPIO.output(GPIO_Ain2, False)
                GPIO.output(GPIO_Bin1, True)
                GPIO.output(GPIO_Bin2, False)
                pwmA.ChangeDutyCycle(60)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(70)
            elif tomato < 1.5 and pixelnum1 > 5000:
                GPIO.output(GPIO_Ain1, True)
                GPIO.output(GPIO_Ain2, False)
                GPIO.output(GPIO_Bin1, False)
                GPIO.output(GPIO_Bin2, True)
                pwmA.ChangeDutyCycle(60)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(70)
            else:
                GPIO.output(GPIO_Ain1, True)
                GPIO.output(GPIO_Ain2, False)
                GPIO.output(GPIO_Bin1, True)
                GPIO.output(GPIO_Bin2, False)
                pwmA.ChangeDutyCycle(60)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(70) 
            
            '''ax.plot((x0, x1), (y0, y1), '-r', linewidth=2.5)
                ax.plot((x0, x2), (y0, y2), '-r', linewidth=2.5)
                ax.plot(x0, y0, '.g', markersize=15)

                minr, minc, maxr, maxc = props.bbox
                bx = (minc, maxc, maxc, minc, minc)
                by = (minr, minr, maxr, maxr, minr)
                ax.plot(bx, by, '-b', linewidth=2.5)

            ax.axis((0, 600, 600, 0))
            plt.show()'''
            
            '''if x1 < 5:
                    GPIO.output(GPIO_Ain1, True)
                    GPIO.output(GPIO_Ain2, False)
                    GPIO.output(GPIO_Bin1, True)
                    GPIO.output(GPIO_Bin2, False)
                    pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                    pwmB.ChangeDutyCycle(100)
                else:
                    GPIO.output(GPIO_Ain1, True)
                    GPIO.output(GPIO_Ain2, False)
                    GPIO.output(GPIO_Bin1, False)
                    GPIO.output(GPIO_Bin2, True)
                    pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                    pwmB.ChangeDutyCycle(100) 
                    '''

    
            # Show the frames
            # The waitKey command is needed to force openCV to show the image
            '''cv2.imshow("Frame", image)
            cv2.imshow("Mask1", mask1)
            cv2.imshow("Mask2", mask2)
            cv2.waitKey(1)
            
            counter = 0
            counter1 = 0
            
            if pixelnum1 >= 5000:
                GPIO.output(GPIO_Ain1, True)
                GPIO.output(GPIO_Ain2, False)
                GPIO.output(GPIO_Bin1, False)
                GPIO.output(GPIO_Bin2, True)
                pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(100)                # duty cycle between 0 and 100
                counter=counter+1
                print ("Forward turn")
                if counter > 20:
                    GPIO.output(GPIO_Ain1, True)
                    GPIO.output(GPIO_Ain2, False)
                    GPIO.output(GPIO_Bin1, True)
                    GPIO.output(GPIO_Bin2, False)
                    pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                    pwmB.ChangeDutyCycle(100)
                else:
                    time.sleep(0.1)
                
                
            elif pixelnum2 >= 5000:
                GPIO.output(GPIO_Ain1, False)
                GPIO.output(GPIO_Ain2, True)
                GPIO.output(GPIO_Bin1, True)
                GPIO.output(GPIO_Bin2, False)
                pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(100)                # duty cycle between 0 and 100
                print ("Forward turn")
                counter1=counter1+1
                if counter1 > 20:
                    GPIO.output(GPIO_Ain1, True)
                    GPIO.output(GPIO_Ain2, False)
                    GPIO.output(GPIO_Bin1, True)
                    GPIO.output(GPIO_Bin2, False)
                    pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                    pwmB.ChangeDutyCycle(100)
                else:
                    time.sleep(0.1)
                
            else:
                GPIO.output(GPIO_Ain1, True)
                GPIO.output(GPIO_Ain2, False)
                GPIO.output(GPIO_Bin1, True)
                GPIO.output(GPIO_Bin2, False)
                pwmA.ChangeDutyCycle(84)                # duty cycle between 0 and 100
                pwmB.ChangeDutyCycle(100)'''
                
                

                                   

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        GPIO.cleanup()
        cv2.destroyAllWindows()
        camera.close()
        
        

