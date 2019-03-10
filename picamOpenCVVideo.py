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


    pic = cv.Canny(img_dilation, 150, 170)

    params = cv.SimpleBlobDetector_Params()


    # Change thresholds
    params.minThreshold = 2# 2
    params.maxThreshold = 500
    params.filterByCircularity = True
    params.minCircularity = 0.3
    params.maxCircularity = 1.5


    detector = cv.SimpleBlobDetector_create(params)

    keypoints = detector.detect(edges)

    # show the frame
    cv.imshow("Frame", pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break
