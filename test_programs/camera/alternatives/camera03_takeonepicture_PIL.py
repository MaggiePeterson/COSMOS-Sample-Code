# This program illustrates how to capture a single image and how to do further processing on it
# The image capture is part of a straight line program
# It uses numpy to do the calculations and PIL to display the images

import picamera
import picamera.array                           # This needs to be imported explicitly
import time
from PIL import Image
import numpy as np



# Initialize the camera
camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.vflip = False                            # Flip upside down or not
camera.hflip = True                             # Flip left-right or not


# Create a data structure to store a frame
rawframe = picamera.array.PiRGBArray(camera, size = (640, 480))


# Allow the camera to warm up
time.sleep(1)


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


# To show the images, we convert to PIL
img0 = Image.fromarray(img_np)
img1 = Image.fromarray(img_np1)
img2 = Image.fromarray(img_np2)
img3 = Image.fromarray(img_np3)
img0.show()
img1.show()
img2.show()
img3.show()


# Save the image to file
img1.save("image_modified.jpg")


# Clean up the resources
camera.close()
        
