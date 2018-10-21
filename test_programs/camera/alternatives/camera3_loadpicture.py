# This is a basic program that illustrates how to read a picture from file
# and convert it into a numpy array
# It is based on PIL


from PIL import Image
import numpy as np


# Read a PIL image from file
img0 = Image.open("test_image.jpg")

# This also reads a PIL image from file and then converts it into grayscale 
img0 = Image.open("test_image.jpg").convert("L")

# Show the PIL image
img0.show()


# Convert the PIL image to a numpy array
# We could do all image manipulations on the PIL image directly
# However, this is slow and it is better to convert the PIL image to a
# numpy array and do all the processing there, and then convert it back
# to a PIL image
img_np = np.array(img0)


# Do something with the image stored in the numpy array
w,h,d = img_np.shape
img_np[w//2-2:w//2+2 , h//2-2:h//2+2 , :] = 0


# Convert the numpy array to a PIL image
img1 = Image.fromarray(img_np)


# Save the PIL image to file
img1.save("test_image_new.jpg")
img1.show()


