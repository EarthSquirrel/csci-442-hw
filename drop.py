import maestro
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np


servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
ARM = 6
HAND = 11

open_hand = 50
closed_hand = -50
raised_arm = -50
lower_arm = 50

servo.setTarget(ARM, raised_arm)
servo.setTarget(HAND, open_hand)
servo.setTarget(HEADTILT,1510)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def detect_ice(raw_img):
    raw_img = raw_img[int(height/2):height, 0:width] # int(width*.25):int(width*.75)]
    blur = cv.blur(raw_img,(3,3))
    # cv.imshow('raw_img', raw_img)
    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=1)

    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)
    cv.imshow("hsv", hsv)
    # robot lab settings
    hsv_min, hsv_max = (0, 0, 0), (70, 70, 90)
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)
    cv.imshow("filter", color_filter)

    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    # cv.imshow('edges', dil_edges)

    contours, hierarchy = cv.findContours(dil_edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    width_i, height_i, channel_i = raw_img.shape
    tours = 255 * np.ones((width_i,height_i,1), np.uint8)
    thresh = raw_img.copy()

    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))

    contoursS = sorted(contours, key=lambda x: myContourArea(x))
    contoursS.reverse()

    if len(contoursS) == 0:
        paused = True

    cx, cy = -1, -1

    for cnt in contoursS:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)
        if width*height < 10:
            print('area is {} too small. No contours found.'.format(x*y))
            paused = True
        M = cv.moments(cnt)
        if M['m00'] == 0:
            div_by =0.1
        else:
            div_by = M['m00']

        cx = int(M['m10']/div_by)
        cy = int(M['m01']/div_by)
        break

    cog = (cx, cy)
    cv.rectangle(thresh, (cx,cy), (cx+29, cy+20),(0,0,255), 2)
    cv.imshow('contours', thresh)

    return hsv




# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    width, height, channel = image.shape

    pic = cv.Canny(image, 100, 170)

    detect_ice(image)

    # show the frame


    cv.imshow("Frame", image)# pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break

