# This program illustrates how to capture a single image and how to do further processing on it
# The image capture is part of a loop
# It uses openCV

import picamera
import picamera.array                           # This needs to be imported explicitly
import time
import cv2



# Initialize the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size = (640, 480))


# Allow the camera to warm up
time.sleep(2)
print("Starting the main program")


if __name__ == '__main__':
    try:

        counter = 0
        
        # Loop
        # Capture an individual frame and put it in an array for processing
        while (counter < 10):

            # Capture a single frame and store it in the array we created before
            # Note that we chose the BGR format, since we will be passing this on to openCV
            camera.capture(rawframe, format = 'bgr')

            # Clear the rawframe in preparation for the next frame
            # This way, you can use the same rawframe for the next image capture if you need to
            # You need this if you want to place the camera.capture in a loop
            rawframe.truncate(0)

            
            # Create a numpy array representing the image   
            image = rawframe.array


            #-----------------------------------------------------
            # We will use numpy and OpenCV for image manipulations
            #-----------------------------------------------------

            # Convert for BGR to HSV color space, using openCV
            # The reason is that it is easier to extract colors in the HSV space
            # Note: this transformation is also why the format for the camera.capture was chosen to be BGR     
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


            # Show the images
            # The waitKey command is needed to force openCV to show the image; if the image does not show up, you can increase the number of ms (the argument of waitKey)
            cv2.imshow("Image in BGR", image)
            cv2.imshow("Image in HSV", image_hsv)
            cv2.waitKey(100)
           

            # Update the loop counter
            counter = counter + 1
            print("Loop %d" % counter)
            time.sleep(0.5)


        # After the while loop, clean up the resources
        cv2.destroyAllWindows()
        camera.close()
        
    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        cv2.destroyAllWindows()
        # Clean up the camera resources
        camera.close()
        
