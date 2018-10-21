# This program illustrates how to capture frames in a video stream and how to do further processing on them
# It uses openCV

import picamera
import picamera.array                           # This needs to be imported explicitly
import time
import cv2



# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))


# Allow the camera to warm up
time.sleep(0.1)


if __name__ == '__main__':
    try:
        
        # Continuously capture frames from the camera
        # Note that the format is BGR instead of RGB because we want to use openCV later on and it only supports BGR
        # However, if we don't use openCV, we also capture as 'rgb'
        for frame in camera.capture_continuous(rawframe, format = 'bgr', use_video_port = True):

            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)

            
            # Create a numpy array representing the image
            image = frame.array     

            #-----------------------------------------------------
            # We will use numpy and OpenCV for image manipulations
            #-----------------------------------------------------
            
            # Convert for BGR to HSV color space, using openCV
            # Note: the fact that we are using openCV is why the format for the camera.capture was chosen to be BGR
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


            # Show the frames
            # The waitKey command is needed to force openCV to show the image
            cv2.imshow("Frame in BGR", image)
            cv2.imshow("Frame in HSV", image_hsv)
            cv2.waitKey(1)





    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        cv2.destroyAllWindows()
        # Clean up the camera resources
        camera.close()
        
