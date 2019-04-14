# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import maestro
import sys
import threading
from client import ClientSocket
import numpy as np
import traceback
import logging

##############################################################################
############################ methods #########################################
##############################################################################


def check_crossed(raw_img):
    # TODO: Check if robot crossed a line into a new state
    print('check_crossed')
    raw_img = raw_img[int(1*height/6):height, 0:width] # int(width*.25):int(width*.75)]
    blur = cv.blur(raw_img,(7,7))

    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=1)
    img_dilation = cv.dilate(img_dilation, kernel, iterations=3)

    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

    # robot lab settings
    hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)

    cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    cv.imshow('edges', dil_edges)

    contours, hierarchy = cv.findContours(dil_edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    width_i, height_i, channel_i = raw_img.shape
    tours = 255 * np.ones((width_i,height_i,1), np.uint8)
    thresh = raw_img.copy()

    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))

    contoursS = sorted(contours, key=lambda x: myContourArea(x))
    contoursS.reverse()

    if len(contoursS) == 0:
        paused = True

    cx, cy = 0, 0

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

    if old_cog_line[0] > cog[0]:
        # moving forward towards line
        return True
    else:
        return False

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h



def print_bool_vals():
    keys = list(bool_vals.keys())
    output = 'Boolean values:\n'
    for key in keys:
        output += '\t' + key + ': ' + bool_vals[key]


def stop():
    global motors, turn, body, headTurn, headTilt
    motors = 6000
    turn = 6000
    body = 6000
    headTurn = 6000
    headTilt = 6000

    servo.setTarget(MOTORS, motors)
    servo.setTarget(TURN, turn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)
    servo.setTarget(BODY, body)


def talk(say_this):
    PORT = 5010
    client = ClientSocket(IP, PORT)
    ##client.start()
    for i in say_this:
        client.sendData(i)
        print('\tspeaking: ', i)
        time.sleep(1)
    client.killSocket()



#############################################################################
######################## INITIALIZE THINGS ##################################
#############################################################################

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
# face_cascade = cv.CascadeClassifier('lbpcascade_frontalface_improved.xml')


# make and initialize motor information
# TODO: Add information for arms here
servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000


max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

# In the hall
max_move = 5600
#max_turn = 6800
#min_turn = 5000

# IP address for talking
IP = '10.200.22.237'

##################################################3##
############## Boolean values #######################
#####################################################

# states
start_field = True
avoidance = False
mining = False
changedState = False

# global accross all states
hasBall = False

# start_filed bools
droppedBall = False

# mining bools
sawHuman = False
getBall = False

old_cog_line = (float('inf'), float('inf'))


# make a dictionary to be able to print values easily
bool_vals = {'start_field': start_field, 'avoidance': avoidance, 'mining': mining,
             'hasBall': hasBall, 'droppedBall': droppedBall, 'sawHuman': sawHuman,
             'getBall': getBall, 'changedState': changedState}

# allow the camera to warmup
time.sleep(0.1)

try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        width, height, channel = image.shape
        # TODO: Check if robot crossed a line into a new state
        print('check_crossed')
        raw_img = raw_img[int(1*height/6):height, 0:width] # int(width*.25):int(width*.75)]
        blur = cv.blur(raw_img,(7,7))

        kernel = np.ones((10,10), np.uint8)

        img_erosion = cv.erode(blur, kernel, iterations=2)
        img_dilation = cv.dilate(img_erosion, kernel, iterations=1)
        img_dilation = cv.dilate(img_dilation, kernel, iterations=3)

        # color filtering stuff, save for later
        hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

        # robot lab settings
        hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
        color_filter = cv.inRange(hsv, hsv_min, hsv_max)

        cv.imshow('hsv',color_filter) # hsv)

        edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
        kernel = np.ones((10,10), np.uint8)
        dil_edges = cv.dilate(edges, kernel, iterations=1)
        cv.imshow('edges', dil_edges)

        contours, hierarchy = cv.findContours(dil_edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        width_i, height_i, channel_i = raw_img.shape
        tours = 255 * np.ones((width_i,height_i,1), np.uint8)
        thresh = raw_img.copy()

        cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))

        contoursS = sorted(contours, key=lambda x: myContourArea(x))
        contoursS.reverse()

        if len(contoursS) == 0:
            paused = True

        cx, cy = 0, 0

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


        # check_crossed(image)
        if start_field:
            if changedState:
                # talk
                speaking = ['Crossed into starting field']
                # talk(speaking)
                print(speaking)
                changedState = False

            if hasBall:
                # drop the ball in the cup
                pass
            elif not hasBall:
                # go to avoidance to get the ball
                pass

        elif avoidance:
            if changedState:
                # talk
                speaking = ['Crossed into starting field']
                # talk(speaking)
                print(speaking)
                changedState = False

            if hasBall:
                # move to start_field
                pass
            elif not hasBall:
                # move to mining
                pass
            else:
                print('this else is silly')

        elif mining:
            if changedState:
                # talk
                speaking = ['Crossed into starting field']
                # talk(speaking)
                print(speaking)
                changedState = False

            if hasBall:
                # scan for the human
                if sawHuman:
                    # talk to human
                    # get the rock
                    pass
                elif not sawHuman:
                    # scan for human
                    pass

            elif not hasBall:
            # scan for the human
                pass

        else:
            print('Error: not a valid state!')

        rawCapture.truncate(0)



except Exception as e: # catch *all* exceptions
    stop()
    print(traceback.print_tb(e.__traceback__))
    print('Stopped motors due to an error')
