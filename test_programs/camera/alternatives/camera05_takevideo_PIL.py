# This program illustrates how to capture frames in a video stream and how to do further processing on them
# It uses numpy to do the calculations and PIL to display the frames
# The use of PIL is inefficient, however, as it opens a new window for each frame

import picamera
import picamera.array                           # This needs to be imported explicitly
from PIL import Image
import time
import numpy as np


# Initialize the camera and grab a reference to the frame
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not


# Create a data structure to store a frame
rawframe = picamera.array.PiRGBArray(camera, size=(640, 480))


# Allow the camera to warm up
time.sleep(0.1)


if __name__ == '__main__':
    try:
        
        # Continuously capture frames from the camera
        # Note that we chose the RGB format
        for frame in camera.capture_continuous(rawframe, format = 'rgb', use_video_port = True):

            # Clear the stream in preparation for the next frame
            rawframe.truncate(0)

            
            # Create a numpy array representing the image
            img_np = frame.array


            #-----------------------------------------------------
            # We will use numpy to do all our image manipulations
            #-----------------------------------------------------

            # Make a copy of the image
            img_np1 = img_np.copy()

            # Modify the copy of the image 
            img_np1.setflags(write=1)                                   # Making the array mutable                                                                                                      
            w,h,d = img_np1.shape
            img_np1[w//4:3*w//4 , h//4:3*h//4 , :] = 255 - img_np1[w//4:3*w//4 , h//4:3*h//4 , :]



            # To show the image, we convert to PIL
            # The disadvantage is that this opens up a new window for each frame
            # We therefore added a delay to make it more manageable
            img0 = Image.fromarray(img_np1)
            img0.show()
            time.sleep(4)
            



    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        # Clean up the camera resources
        camera.close()
        
