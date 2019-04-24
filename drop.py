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
ELBOW = 6
TWIST = 7

open_hand = 4000
closed_hand = 7400
raised_arm = 8000
lower_arm = 4000
elbow_straight = 6000
twist_in = 5000

servo.setTarget(ARM, raised_arm)
servo.setTarget(HAND, open_hand)
servo.setTarget(TWIST, twist_in)
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

def detect_ice(raw_img):
    width, height, channel = raw_img.shape
    width_array = len(raw_img[0])
    raw_img = raw_img[0:height,int(width_array/2):width_array]#int(width/3):width] # int(width*.25):int(width*.75)]

    # yellow_min = np.array([100, 0, 150])
    # yellow_max = np.array([135, 15, 255])

    hsv_min, hsv_max = (50,0,150), (135,50,255)
    color_filter = get_hsv_filter(raw_img, hsv_min, hsv_max)
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


    cx, cy = width*100, height*100
    if len(contoursS) > 0:
        center_contour = (10000000, contoursS[0], (cx, cy))
    width, height, channel = raw_img.shape
    for cnt in contoursS:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)
        if w*h > 500:
            # print('area is {} too small. No contours found.'.format(x*y))

            M = cv.moments(cnt)
            if M['m00'] == 0:
                div_by =0.1
            else:
                div_by = M['m00']

            cx = int(M['m10']/div_by)
            cy = int(M['m01']/div_by)

            cog = (cx, cy)
            temp_dist =np.linalg.norm(np.array(cog) - np.array((width/2, height/2)))
            if temp_dist < center_contour[0]:
                center_contour = (temp_dist, cnt, cog)

            cv.rectangle(thresh, (cx,cy), (cx+29, cy+20),(0,0,255), 2)
    if len(contoursS) > 0:
        cv.rectangle(thresh, center_contour[2], (cx+29, cy+20),(0,255,255), 2)
    cv.rectangle(thresh, (int(width/2), int(height/2)), (cx+29, cy+20),(0,255,255), 2)

    cv.imshow('contours', thresh)





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
    if key == ord('c'):
        cv.imwrite('drop-image.png', image)
