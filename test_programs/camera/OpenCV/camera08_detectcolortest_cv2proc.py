# This program illustrates how to capture a images as part of a video stream
# and how to do extract pixels of a specific color
# It includes a slider to adjust the color that is being filtered
# It uses openCV

import cv2
import picamera
import picamera.array                                   # This needs to be imported explicitly
import time
import numpy as np

def nothing(x):
    pass

# Create a window for later use by the track bar
cv2.namedWindow('result1')
h1, s1, v1 = 100, 100, 100
img_low = np.zeros((15,512,3),np.uint8)

cv2.namedWindow('result2')
h2, s2, v2 = 100, 100, 100
img_high = np.zeros((15,512,3),np.uint8)


# Create a track bar
cv2.createTrackbar('h1','result1', 0, 255, nothing)
cv2.createTrackbar('s1','result1', 0, 255, nothing)
cv2.createTrackbar('v1','result1', 0, 255, nothing)

cv2.createTrackbar('h2','result2', 0, 255, nothing)
cv2.createTrackbar('s2','result2', 0, 255, nothing)
cv2.createTrackbar('v2','result2', 0, 255, nothing)


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
            cv2.imshow('result1', img_low)
            h1 =  cv2.getTrackbarPos('h1','result1')
            s1 =  cv2.getTrackbarPos('s1','result1')
            v1 =  cv2.getTrackbarPos('v1','result1')
            img_low[:] = [h1,s1,v1]
            
            cv2.imshow('result2', img_high)
            h2 =  cv2.getTrackbarPos('h2','result2')
            s2 =  cv2.getTrackbarPos('s2','result2')
            v2 =  cv2.getTrackbarPos('v2','result2')
            img_high[:] = [h2,s2,v2]
            
            # Define the range colors to filter; these numbers represent HSV
            lowerColorThreshold = np.array([h1,s1,v1])
            upperColorThreshold = np.array([h2,s2,v2])
            #upperColorThreshold = np.array([255, 255, 255])

            # Threshold the HWV image to get only colors in a range
            # The colors in range are set to white (255), while the colors not in range are set to black (0)
            mask = cv2.inRange(image_hsv, lowerColorThreshold, upperColorThreshold)

            # Bitwise AND of the mask and the original image
            image_masked = cv2.bitwise_and(image, image, mask = mask)

    
            # Show the frames
            # The waitKey command is needed to force openCV to show the image
            cv2.imshow("Frame", image)
            cv2.imshow("Mask", mask)
            cv2.imshow("Res", image_masked)
            cv2.waitKey(1)



    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        cv2.destroyAllWindows()
        camera.close()
        
