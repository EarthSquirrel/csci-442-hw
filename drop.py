import maestro
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import threading


servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
ARM = 6
HAND = 10
ELBOW = 8
TWIST = 7
WRIST = 11

open_hand = 4000
closed_hand = 5600
raised_arm = 8000
lower_arm = 4000
elbow_straight = 6000
twist_in = 5000
wrist = 6000

servo.setTarget(ARM, raised_arm)
servo.setTarget(HAND, open_hand)
servo.setTarget(TWIST, twist_in)
servo.setTarget(WRIST, wrist)
servo.setTarget(HEADTILT,1510)

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h

def get_hsv_filter(raw_img, hsv_min, hsv_max):
    blur = cv.blur(raw_img,(5,5))
    # cv.imshow('raw_img', raw_img)
    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=2)

    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

    # robot lab settings
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)
    return color_filter

counting_ice_cnts = False
counting_ice_cnts_timer = 0
hasBall = False
rejectedPink = False

def cnts_ice_timer():
    global counting_ice_cnts_timer
    counting_ice_cnts_timer += 1
    print('\t\tContours ice timer: {}'.format(counting_ice_cnts_timer))
    if counting_ice_cnts:
        threading.Timer(1,cnts_ice_timer).start()

def get_contours(edges):
    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
    contoursS = sorted(contours, key=lambda x: myContourArea(x))
    contoursS.reverse()
    return contoursS

def detect_ice(raw_img):
    global counting_ice_cnts, counting_ice_cnts_timer, hasBall, rejectedPink
    width, height, channel = raw_img.shape
    width_array = len(raw_img[0])
    raw_img = raw_img[int(height/6):int(2*height/3)-20,int(width_array/2)+100:width_array-50]#int(width/3):width] # int(width*.25):int(width*.75)]


    hsv_min, hsv_max = (40, 200, 200), (50, 240, 240)
    ice_filter = get_hsv_filter(raw_img, hsv_min, hsv_max)
    # cv.imshow("ice", ice_filter)

    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(ice_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    ice_cnts = get_contours(dil_edges)

    if counting_ice_cnts:
        if counting_ice_cnts_timer > 3:
            counting_ice_cnts = False
            print('GRAB THE ICE!!!')
            servo.setTarget(HAND, closed_hand)
            hasBall = True
            #TODO: grab the ice
    elif len(ice_cnts) > 0 and not hasBall:
        print('I will take that')
        counting_ice_cnts = True
        counting_ice_cnts_timer = 0
        threading.Timer(1, cnts_ice_timer).start()

    thresh = raw_img.copy()
    for cnt in ice_cnts:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,0,255), 2)

    # these are pink colors
    hsv_min, hsv_max = (150, 100, 230), (170, 140, 255)
    pink_filter = get_hsv_filter(raw_img, hsv_min, hsv_max)

    edges = cv.Canny(pink_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    green_cnts = get_contours(dil_edges)
    if len(green_cnts) > 0 and not rejectedPink:
        print('Not the pink, green ice please')
        rejectedPink = True
    for cnt in green_cnts:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (255, 0,0), 2)
    cv.imshow('ice seen', thresh)




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
    #if hasBall:
    #    break

    cv.imshow("Frame", image)# pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
            break
    if key == ord('c'):
        cv.imwrite('drop-image.png', image)
