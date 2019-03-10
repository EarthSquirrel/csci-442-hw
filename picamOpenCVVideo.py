# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 15  #   32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)


# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    width, height, channel = img.shape
    img = img[int(1*height/3):height, 20:width-20]
    # __,new_image = cv.threshold(img, 0, 400, cv.THRESH_BINARY)
    # cv.imshow("newimg", new_image)
    blur = cv.blur(img,(7,7))

    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=3)
    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)
    # hsv = img_dilation.copy()

    hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)
    # pic = cv.Canny(hsv, 150, 170)

    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    cv.imshow('edges', edges)

    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    width, height, channel = img.shape
    tours = 255 * np.ones((width,height,1), np.uint8)
    thresh = img.copy()

    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
    # print(len(contours))
    # contours = cntsSorted[-3]

    for cnt in contours:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)

    cv.drawContours(tours, contours, -1, (0,0,255), -1)
    cv.imshow('Contours', thresh)




    # show the frame
    # cv.imshow("Frame", pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break
