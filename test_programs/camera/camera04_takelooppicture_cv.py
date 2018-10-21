# This program illustrates how to capture a single image and how to do further processing on it
# The image capture is part of a loop
# It uses numpy to do the calculations and OpenCV to display images

import picamera
import picamera.array                           # This needs to be imported explicitly
import time
import cv2
import numpy as np                              



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
            # Note that we chose the RGB format
            camera.capture(rawframe, format = 'rgb')

            # Clear the rawframe in preparation for the next frame
            # This way, you can use the same rawframe for the next image capture if you need to
            # You need this if you want to place the camera.capture in a loop
            rawframe.truncate(0)

            
            # Create a numpy array representing the image   
            img_np = rawframe.array


            #-----------------------------------------------------
            # We will use numpy to do all our image manipulations
            #-----------------------------------------------------
           
            # Here, we create a separate copy of the numpy array because the numpy
            #  array is by default immutable
            w,h,d = img_np.shape
            img_np1 = img_np.copy()
            img_np1[w//4:3*w//4 , h//4:3*h//4 , :] = 0


            # This is a repeat of the functionality above.
            # However, now we are making the array mutable. The resulting operating on
            # the array is now a mutation, also modifying the original copy.
            img_np2 = img_np.copy()
            img_np3 = img_np2
            img_np3.setflags(write=1)
            img_np3[w//4:3*w//4 , h//4:3*h//4 , :] = 255



            # Show the images
            # Note that OpenCV assumes BRG color representation, and we therefore swapped the r and b color channels
            # The waitKey command is needed to force openCV to show the image; if the image does not show up, you can increase the number of ms (the argument of waitKey)
            cv2.imshow("Original frame", img_np[:,:,::-1])
            cv2.imshow("Modified frame", img_np1[:,:,::-1])
            cv2.imshow("Orginal frame copy", img_np2[:,:,::-1])
            cv2.imshow("Modified frame of copy", img_np3[:,:,::-1])
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
        
