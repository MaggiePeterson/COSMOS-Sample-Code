# This program illustrates how to capture a single image and how to do further processing on it
# It uses PIL and numpy

import picamera
import picamera.array                           # This needs to be imported explicitly
from PIL import Image
import numpy as np
import time


# Initialize the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)


# Create an array to store a frame
rawframe = picamera.array.PiRGBArray(camera, size = (640, 480))


# Allow the camera to warm up
time.sleep(1)



if __name__ == '__main__':
    try:

        keepGoing = True
        
        # Loop
        # Capture an individual frame and put it in an array for processing
        while keepGoing:

            # Capture a single frame and store it in the array we created before
            # Note that we chose the RGB format
            camera.capture(rawframe, format = 'rgb')

            # Create a numpy array representing the image   
            img_np = rawframe.array
           
            # To show the image, we convert to PIL
            img0 = Image.fromarray(img_np)
            img0.show()

            # Do something with the image stored in the numpy array
            w,h,d = img_np.shape
            img_np2 = img_np.copy()            
            img_np2[w//4:3*w//4 , h//4:3*h//4 , :] = 0

            # Convert the modified image to PIL
            img1 = Image.fromarray(img_np2)
            img1.show()


            # Clear the stream
            # This way, you can use the same rawframe for the next image capture
            rawframe.truncate(0)

            # Set to 'True' if you want to keep going forever
            keepGoing = False
            if (keepGoing):
                time.sleep(3)


    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Program stopped by User")
        # Clean up the camera resources
        camera.close()
        
