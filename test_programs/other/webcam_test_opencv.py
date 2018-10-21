# This code works mostly. It shows the image from the webcam, but
# does not close the window properly

import cv2
cam=cv2.VideoCapture(0)
img=cam.read()


cv2.namedWindow("camera", cv2.CV_WINDOW_AUTOSIZE)
#cv2.startWindowThread()

cv2.imshow("camera",img[1])
cv2.waitKey(0)
cv2.destroyWindow("camera")
#cv2.destroyAllWindows()


