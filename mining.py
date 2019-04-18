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




# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
# face_cascade = cv.CascadeClassifier('lbpcascade_frontalface_improved.xml')

##############################################################################
############################ methods #########################################
##############################################################################


def check_crossed(raw_img):
    global old_cog_line
    # TODO: Check if robot crossed a line into a new state
    raw_img = raw_img[0:int(height/3), int(width/4):int(2*width/4)] # int(width*.25):int(width*.75)]
    blur = cv.blur(raw_img,(7,7))
    # cv.imshow('raw_img', raw_img)
    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=1)
    img_dilation = cv.dilate(img_dilation, kernel, iterations=3)

    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

    # robot lab settings
    hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)

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
    """
    if old_cog_line[1] > cog[1]:
        # moving forward towards line
        print('True old: {} new: {}'.format(old_cog_line, cog))
        old_cog_line = cog
        return True
    else:
        print('False: old {} new {}'.format(old_cog_line, cog))
        old_cog_line = cog
        return False
    """
    if old_cog_line[1]> 0 and cog[1] == -1:
        # moving forward towards line
        # print('True old: {} new: {}'.format(old_cog_line, cog))
        old_cog_line = cog
        # print('\n\n crossed line!!!!!')
        return True
    else:
        # print('False: old {} new {}'.format(old_cog_line, cog))
        old_cog_line = cog
        return False
    return hsv

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h



def print_bool_vals():
    keys = list(bool_vals.keys())
    output = 'Boolean values:\n'
    for key in keys:
        output += '\t' + key + ': ' + bool_vals[key]


def search_for_face(raw_img):
    # TODO: COpy code from greet_human mostly here

    pass

def stop():
    print('in stop: stopping motors')
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


def go_straight():
    global motors, turn, max_move
    # print('straight')
    servo.setTarget(TURN, 6000)
    motors = max_move
    servo.setTarget(MOTORS, motors)

def search():
    global increasing, headTurn, headTilt, tilt_loc, searching
    if END_PROGRAM:
        stop()
        return
    print('\t\tsearching.....')
    if increasing:
        headTurn += 200
        if headTurn > max_head_turn:
            headTurn = max_head_turn
            increasing = False
    else:
        headTurn -= 200
        if headTurn < min_head_turn:
            headTurn = min_head_turn
            increasing = True
            """
            tilt_loc += 1
            if tilt_loc > 2:
                # maxed out array, return to o
                tilt_loc= 0
            headTilt = tilt_positions[tilt_loc]
            """
    # print("\t\t\tSearching: Head Pos: ", headTurn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)

    if not sawHuman and searching:
       threading.Timer(move_wait_time, search).start()
    else: # no longer searching if found a face
        searching = False

# greater than 6000
def turn_left(weight):
# def turn_right():
    global motors, turn
    # print('right')
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    # if under it's going right, so switch to left
    if turn < 6000:
        turn = 6000
    turn += int(weight*200)+100
    if turn > max_turn:
        turn = max_turn
    servo.setTarget(TURN, turn)

# less than 6000
def turn_right(weight):
# def turn_left(): # Is this a problem????
    global motors, turn
    # print('left')
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    if turn > 6000:
        turn = 6000
    # increment the turn
    turn -= (int(weight*200)+200)
    # check if it's less than min turn, then set to min turn
    if turn < min_turn:
        turn = min_turn
    servo.setTarget(TURN, turn)
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



# make and initialize motor information
# TODO: Add information for arms here
servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
LEFTARM = 5
LEFTHAND = 10

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000


max_move = 5400  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
min_head_tilt = 1510
max_time = 5
min_arm = 0
max_arm = 200
min_hand = -50
max_hand = 50


# In the hall
#max_move = 5600
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
old_cog_line = (float('inf'), float('inf'))

# mining bools
sawHuman = False
getBall = False
searching = False
increasing = False
move_wait_time = 1.0  # time to wait before moving to new position head

# other
END_PROGRAM = False

# make a dictionary to be able to print values easily
bool_vals = {'start_field': start_field, 'avoidance': avoidance, 'mining': mining,
             'hasBall': hasBall, 'droppedBall': droppedBall, 'sawHuman': sawHuman,
             'getBall': getBall, 'changedState': changedState}

# allow the camera to warmup
time.sleep(0.1)

headTilt = 4000
servo.setTarget(HEADTILT, headTilt)
# go_straight()
avoidance = True
start_field = False

try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        width, height, channel = image.shape

        # Change the state if crossed a line
        if not searching and check_crossed(image):
            changedState = True
            # TODO: Check this logic works on the field
            if start_field:
                start_field = False
                avoidance = True
            elif avoidance:
                avoidance = False
                mining=True
                # shoot me please
                pass
            elif mining:
                mining = False
                avoidance = True
            else:
                print('not in any state, there"s a problem!')

        if start_field:
            if changedState:
                # headTilt = min_head_tilt
                servo.setTarget(HEADTILT, headTilt)
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
                speaking = ['Crossed into avoidance']
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
                sawHuman = False
                # talk
                speaking = ['Crossed into mining']
                # talk(speaking)
                print(speaking)
                changedState = False

            if hasBall:
                # scan for the human
                if sawHuman:
                    # talk to human
                    # get the rock
                    pass

            elif not hasBall:
            # scan for the human
                if not sawHuman:
                    # if searching hasn't started, start
                    if not searching:
                        stop()
                        searching = True
                        headTilt = 6000
                        servo.setTarget(HEADTILT, headTilt)
                        threading.Timer(move_wait_time, search).start()
                elif sawHuman:
                    # get the ball
                    pass

        else:
            print('Error: not a valid state!')
        key = cv.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            stop()
            END_PROGRAM = True
            break
        if key == ord('p'):
            stop()

except KeyboardInterrupt:
    print('Manually stopped')
    END_PROGRAM = True
    stop()

except Exception as e: # catch *all* exceptions
    stop()
    END_PROGRAM = True
    print(traceback.print_tb(e.__traceback__))
    print('Stopped motors due to an error')
except:
    print('Something didn"t catch....')
    END_PROGRAM = True
    stop()
