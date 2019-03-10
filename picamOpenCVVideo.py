# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    blur = cv.blur(img,(3,3))

    kernel = np.ones((5,5), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=1)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=1)
    # color filtering stuff, save for later
    edges = cv.Canny(img_dilation, 35, 150, L2gradient=True)

    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    width, height, channel = img.shape
    tours = 255 * np.ones((width,height,1), np.uint8)

    cv.drawContours(tours, contours, -1, (0,0,255), -1)
    cv.imshow('edges', tours)

    """
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

    hsv_min, hsv_max = (0, 50, 50), (100, 250, 255)
    color_filter = cv.inRange(img_dilation, hsv_min, hsv_max)
    # pic = cv.Canny(hsv, 150, 170)

    cv.imshow('hsv', hsv) # color_filter) # hsv)
    """



    # show the frame
    # cv.imshow("Frame", pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break
