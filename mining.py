# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import maestro
import threading
from client import ClientSocket
import numpy as np
import traceback
import sys

##############################################################################
#############################################################################
######################### RIP MR. ROBOT #####################################
##############################################################################
##############################################################################


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
camera.start_preview()
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')
# face_cascade = cv.CascadeClassifier('lbpcascade_frontalface_improved.xml')

##############################################################################
############################ methods #########################################
##############################################################################


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

def get_contours(edges):
    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
    contoursS = sorted(contours, key=lambda x: myContourArea(x))
    contoursS.reverse()
    return contoursS

def check_crossed(raw_img,  hsv_min, hsv_max, window_name='check crossed'):
    global pink_cog_line, yellow_cog_line
    raw_img = raw_img[int(height/2):height, int(width/5):int(4*width/5)] # int(width*.25):int(width*.75)]

    blur = cv.blur(raw_img,(5,5))
    # cv.imshow('raw_img', raw_img)
    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=3)
    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)

    # robot lab settings
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)

    #color_filter = get_hsv_filter(raw_img, hsv_min, hsv_max)
    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=3)
    # cv.imshow('edges', dil_edges)
    edge_name = window_name + '-edges'
    #cv.imshow(edge_name, dil_edges)

    thresh = raw_img.copy()

    contoursS = get_contours(dil_edges)

    if len(contoursS) == 0:
        paused = True

    cx, cy = -1, -1

    for cnt in contoursS:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)
        #print(w*h)
        if w > 0: # width/4:
            paused = True
            M = cv.moments(cnt)
            if M['m00'] == 0:
                div_by =0.1
            else:
                div_by = M['m00']

            cx = int(M['m10']/div_by)
            cy = int(M['m01']/div_by)
            break
        else:
            print('area is {} too small for {}. No contours found.'.format(w, width/2))

    cog = (cx, cy)
    cv.rectangle(thresh, (cx,cy), (cx+29, cy+20),(0,0,255), 2)
    cv.imshow(window_name, thresh)

    if window_name == 'pink line':
        old_cog_line = pink_cog_line
    else:
        old_cog_line = yellow_cog_line

    # print(old_cog_line, ' ' , window_name)
    #if old_cog_line[1][1]> 0 and cog[1] == -1 and old_cog_line[0][1] == -1:
    if old_cog_line[1]> 0 and cog[1] == -1:
        # moving forward towards line
        # print('True old: {} new: {}'.format(old_cog_line, cog))
        if window_name == 'pink line':
            #pink_cog_line [1] = old_cog_line[0]
            #pink_cog_line[0] = cog
            pink_cog_line = cog
        else:
            #yellow_cog_line [1] = old_cog_line[0]
            #yellow_cog_line[0] = cog
            yellow_cog_line = cog

        return True
    else:
        if window_name == 'pink line':
            #pink_cog_line [1] = old_cog_line[0]
            #pink_cog_line[0] = cog
            pink_cog_line = cog
        else:
            #yellow_cog_line [1] = old_cog_line[0]
            #yellow_cog_line[0] = cog
            yellow_cog_line = cog
        return False


def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h



def print_bool_vals():
    keys = list(bool_vals.keys())
    output = 'Boolean values:\n'
    for key in keys:
        output += '\t' + key + ': ' + bool_vals[key]


def chase_human(image):
    # move twoards the human, keeping face
    pass


def time_the_faces():
    if END_PROGRAM:
        return
    global face_timer
    face_timer += 1
    print('\t\tface_timer: ', face_timer)
    if mining and not gettingIce:
        threading.Timer(1,time_the_faces).start()

def load_images_clock():
    global load_images_timer, load_images
    if END_PROGRAM:
        stop()
        return
    load_images_timer += 1
    # print('\t\tload_images_timer ', load_images_timer)
    if load_images_timer < 8:
        threading.Timer(1, load_images_clock).start()
    else:
        load_images = False
        go_straight()

def get_turn_dir():
    global turn_dir
    head_pos = servo.getPosition(HEADTURN)
    face_loc = faces[0][0] + faces[0][2]/2
    # if (head_pos < 6000 and face_loc < width/2) or (head_pos > 6000 and face_loc < width/2):
    if head_pos >= 6000:
        print("Turn dir = right")
        turn_dir = "right"
    # elif (head_pos > 6000 and face_loc > width/2) or (head_pos < 6000 and face_loc > width/2):
    if head_pos < 6000:
        print("Turn dir = left")
        turn_dir = 'left'


def stop_face_center(width, faces):
    global repositioning, gotoHuman
    face_center = faces[0][0] + 0.5*faces[0][2]
    if abs(face_center - width/2) < width/5:
        print('Face in center, stoping things')
        pause()
        repositioning = False
        gotoHuman = True
        face_timer = 0

def reposition(turn_dir):
    global turn
    motors = 6000
    if turn_dir == 'left':
        print('\treposition to the left')
        # turn -= 200
        turn = min_turn
        if turn < min_turn:
            turn = min_turn
    elif turn_dir == 'right':
        print('\treposition to the right')
        # turn += 200
        turn = max_turn
        if turn > max_turn:
            turn = max_turn
    else:
        print('Problem!')
        # print('\tYa, something broke....')
        #motors = max_move - 200
        #servo.setTarget(MOTORS, motors)
        #time.sleep(0.15)
        #motors = max_move +200

    servo.setTarget(TURN, turn)
    # servo.setTarget(MOTORS, motors)
    time.sleep(0.25)
    # print('\trepositioning turn value: ' + str(servo.getPosition(TURN)))
    turn = 6000
    motors = 6000
    servo.setTarget(MOTORS, motors)
    servo.setTarget(TURN, turn)
    time.sleep(.5)

def faces_found(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, minSize=(10,10), maxSize=(width, height))
    servo.setTarget(TURN, turn)
    real_faces = []
    # if len(faces) > 0:
        # print('Faces total found: {}'.format(len(faces)))
    for (x,y,w,h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        # don't worry about eyes if chacing human, need to get it!
        eyes = eye_cascade.detectMultiScale(roi_gray)
        # If there are no eyes1111, don't use this face
        if len(eyes) == 0:
            continue
        #Otherwise, we use it and return it to our list of return faces
        # print('total eye count: {}'.format(len(eyes)))
        real_faces.append((x,y,w,h))
        # cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    if len(real_faces) > 0:
        print('\tfaces_found has a face!')
    return real_faces


def cnts_ice_timer():
    global counting_ice_cnts_timer
    counting_ice_cnts_timer += 1
    print('\t\tContours ice timer: {}'.format(counting_ice_cnts_timer))
    if counting_ice_cnts:
        threading.Timer(1,cnts_ice_timer).start()

def detect_ice(raw_img):
    global counting_ice_cnts, counting_ice_cnts_timer, hasBall, rejectedPink
    width, height, channel = raw_img.shape
    width_array = len(raw_img[0])
    raw_img = raw_img[int(height/6):int(2*height/3)-20,int(width_array/2)+100:width_array-50]#int(width/3):width] # int(width*.25):int(width*.75)]


    ice_filter = get_hsv_filter(raw_img, green_ice_min, green_ice_max)
    # cv.imshow("ice", ice_filter)

    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(ice_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    ice_cnts = get_contours(dil_edges)

    if counting_ice_cnts:
        if counting_ice_cnts_timer > 1:
            counting_ice_cnts = False
            print('GRAB THE ICE!!!')
            servo.setTarget(HAND, closed_hand)
            hasBall = True
    elif len(ice_cnts) > 0 and not hasBall:
        speak = ['I will take that']
        print(speak)
        if talking:
            talk(speak)

        counting_ice_cnts = True
        counting_ice_cnts_timer = 0
        threading.Timer(1, cnts_ice_timer).start()

    thresh = raw_img.copy()
    for cnt in ice_cnts:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,0,255), 2)

    # these are pink colors
    # pink_ice_min, pink_ice_max = (150, 100, 230), (170, 140, 255)
    pink_filter = get_hsv_filter(raw_img, pink_ice_min, pink_ice_max)

    edges = cv.Canny(pink_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    green_cnts = get_contours(dil_edges)
    if len(green_cnts) > 0 and not rejectedPink:
        speak = ['Not the pink, green ice please']
        print(speak)
        if talking:
            talk(speak)

        rejectedPink = True
    for cnt in green_cnts:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (255, 0,0), 2)
    cv.imshow('ice seen', thresh)


def arm_down():
    # set arm so don't have to deal with it later
    servo.setTarget(TWIST, twist_center)
    time.sleep(.25)
    servo.setTarget(HAND, open_hand)
    servo.setTarget(ELBOW, elbow_straight)
    servo.setTarget(WRIST, wrist)
    servo.setTarget(ARM, lower_arm)


def open_hand():
    servo.setTarget(11, 4000)


def close_hand():
    servo.setTarget(HAND, closed_hand)




def stop():
    # print('in stop: stopping motors')
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

def pause():
    # print('in stop: stopping motors')
    global motors, turn, body, headTurn, headTilt
    motors = 6000
    turn = 6000
    body = 6000

    servo.setTarget(MOTORS, motors)
    servo.setTarget(TURN, turn)
    servo.setTarget(BODY, body)

def go_straight():
    global motors, turn, max_move
    # print('straight')
    servo.setTarget(TURN, 6000)
    motors = max_move
    servo.setTarget(MOTORS, motors)

def search():
    global increasing, headTurn, headTilt, tilt_loc, searching
    pause()
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
    #servo.setTarget(HEADTILT, headTilt)

    if not sawHuman and searching:
       threading.Timer(move_wait_time, search).start()
    elif not blob_found and searching:
        threading.Timer(1.0, search).start()
    else: # no longer searching if found a face
        searching = False

def detect_green_box(image, hsv_min, hsv_max):
    hsv = get_hsv_filter(image, hsv_min, hsv_max)

    edges = cv.Canny(hsv, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    # cv.imshow('edges', dil_edges)

    thresh = image.copy()

    contours = get_contours(dil_edges)

    for cnt in contours:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (255, 0,0), 2)

    cv.imshow('Search for green', thresh)
    if len(contours) > 0:
        return True
    else:
        return False

# TODO: TEst this logic
def which_center_green(image, hsv_min, hsv_max):
    h_img, w_img, c = image.shape
    center = int(w_img/2)

    thresh = image.copy()

    hsv = get_hsv_filter(image, hsv_min, hsv_max)
    edges = cv.Canny(hsv, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    contours = get_contours(dil_edges)

    for cnt in contours:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (255, 0,0), 2)
        cv.imshow('Search for green', thresh)
        if abs(x - center) <= w_img/6:
            print('Green is in the center!!!!')
            return 'center'
        else:
            print('Green is not in center of screen')
            if x - center < 0:
                print("it's on the left side!")
                # contour on left side of scree
                return 'right'
            else:
                print("it's on the right side")
                return 'left'
    return 'none'


    # find which side of the image the green box is on, th
    return 'left'

def search_turn():
    if END_PROGRAM:
        stop()
        return
    turn = max_turn
    servo.setTarget(TURN, turn)
    time.sleep(.25)
    turn = 6000
    motors = 6000
    servo.setTarget(MOTORS, motors)
    servo.setTarget(TURN, turn)
    time.sleep(.25)
    if searchingTurn:
        print('search_turn')
        threading.Timer(.25, search_turn).start()


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
    ##client.start()
    for i in say_this:
        client.sendData(i)
        print('\tspeaking: ', i)
        time.sleep(1)

def prepareImage(img):
    kernel = np.ones((10,10), np.uint8)
    blur = cv.blur(img,(7,7))
    erosion = cv.erode(blur, kernel, iterations=4)
    dilation = cv.dilate(erosion, kernel, iterations=2)
    return dilation


def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def find_contours(color, img):
    prep_img = prepareImage(img)
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv_img = prepareImage(hsv_img)
    thresh = cv.inRange(hsv_img, green_ice_min, green_ice_max)
    thresh = prepareImage(thresh)
    edges = cv.Canny(thresh, 35, 150, L2gradient=True)
    edges = cv.blur(edges, (7,7))

    if color == "green":
        min_hsv = green_ice_min
        max_hsv = green_ice_max

    filtered_img = cv.inRange(hsv_img, min_hsv, max_hsv)
    edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)

    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contoursS = sorted(contours, key=lambda x: contourArea(x))

    return contoursS, len(contoursS)


def get_rect_center(rect):
    x,y,w,h = rect
    return np.array([(x+(w/2)), (y+(h/2))])





#############################################################################
######################## INITIALIZE THINGS ##################################
#############################################################################



# make and initialize motor information
# TODO: Add information for arms here
servo = maestro.Controller()

MOTORS =  2# 1
TURN = 1 # 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
ARM = 6
HAND = 11
ELBOW = 8
TWIST = 7
WRIST = 10

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000

max_move = 5200  # 5400
max_turn = 7200#7200  # left
min_turn = 4800#4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
min_head_tilt = 1510
max_time = 5

open_hand = 4000
closed_hand = 6000
raised_arm = 8000
lower_arm = 4000
elbow_straight = 6000
twist_in = 5400
twist_out = 7000
twist_center = 6000
wrist = 8000

# set arm so don't have to deal with it later
servo.setTarget(ARM, raised_arm)
time.sleep(.5)
servo.setTarget(HAND, open_hand)
servo.setTarget(TWIST, twist_out)
servo.setTarget(WRIST, wrist)
servo.setTarget(ELBOW, elbow_straight)

# In the hall
#max_move = 5600
#max_turn = 6800
#min_turn = 5000

# IP address for talking
IP = '10.200.4.23' #23.235'

##################################################3##
############## Boolean values #######################
#####################################################
talking = True
if talking:
    PORT = 5010
    client = ClientSocket(IP, PORT)
previous_state = 'start_field'
# states
start_field = True
avoidance = False
mining = False
changedState = False

# global accross all states
hasBall = False
load_images = True
load_images_timer = 0

# start_field bools
droppedBall = False
blob_found = False
#yellow_cog_line = [(float('inf'), float('inf')), (float('inf'), float('inf'))]
#pink_cog_line = [(float('inf'), float('inf')), (float('inf'), float('inf'))]
yellow_cog_line = (float('inf'), float('inf'))
pink_cog_line = (float('inf'), float('inf'))
go_to_bucket = False

# mining bools
sawHuman = False
getBall = False
searching = False
increasing = False
repositioning = False
gotoHuman = False
stopped = False
turn_dir = 'boom'
move_wait_time = 1.0  # time to wait before moving to new position head
face_timer = 0
doom_timer = 3 # how long to wait when losing a face before stopping
counting_ice_cnts = False  # checks if has seen the ice or not
counting_ice_cnts_timer = 0  # must see the ice for certian amount of tiime, thne close hand
rejectedPink = False
askedForIce = False
gettingIce = False
searchingTurn = False
locatedGreenBox = False
moveToGreen = False

#

# other
END_PROGRAM = False

# make a dictionary to be able to print values easily
bool_vals = {'start_field': start_field, 'avoidance': avoidance, 'mining': mining,
             'hasBall': hasBall, 'droppedBall': droppedBall, 'sawHuman': sawHuman,
             'getBall': getBall, 'changedState': changedState}

# allow the camera to warmup
time.sleep(0.1)
camera.stop_preview()
green_ice_min = np.array([20, 150, 120])
green_ice_max = np.array([100, 255, 220])
#green_ice_min = np.array([20, 160, 120])
#green_ice_max = np.array([75, 225, 190])

#green_ice_min, green_ice_max = (40, 200, 200), (50, 240, 240)
green_min, green_max = (20, 160, 120), (75, 225, 190)
#pink_ice_min, pink_ice_max = (150, 100, 230), (170, 140, 255)
pink_ice_min, pink_ice_max = (140, 10, 125), (170, 20, 200)

# on teh test paper
#yellow_min, yellow_max = (10, 180, 130), (30, 200, 150)
pink_line_min, pink_line_max = (164, 65, 252), (166, 75, 255)
pink_line_min, pink_line_max = (0, 0, 245), (5,5, 255)
pink_line_min, pink_line_max = (135,20,190), (179,255,255) # Peter's

# headTilt = 4000
servo.setTarget(HEADTILT, headTilt)
servo.setTarget(HEADTILT, 1510)
threading.Timer(1, load_images_clock).start()
# TEMPORARY BOOLEAN VARIABLES FOR TESTING DIFFERENT STATES
# go_straight()
# avoidance = True
#start_field = False
#mining = False
#hasBall= True
#changedState = True

#searching = True
#search()

#time.sleep(2.0)
#close_hand()

print('Starting the program')
try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        height, width, channel = image.shape
        cv.imshow("Image", image)

        # yellow

        # Change the state if crossed a line

        if not searching and not searchingTurn and check_crossed(image, pink_line_min, pink_line_max, 'pink line') and not load_images:
            print('previous_state: ', previous_state)
            print('Start {}, avoid {} mine {}'.format(start_field, avoidance, mining))
            if start_field and previous_state == 'start_field':
                changedState = True
                start_field = False
                avoidance = True
            elif avoidance and previous_state == 'mining':
                changedState = True
                avoidance = False
                start_field = True
            elif avoidance and previous_state == 'start_field':
                changedState = True
                avoidance = False
                mining = True
                previous_state = 'avoidance'
            elif mining and previous_state == 'avoidance' and moveToGreen:
                changedState = True
                mining = False
                avoidance = True
                previous_state = 'mining'
            else:
                print('entered changing state logic, but didnt change states.')
            print('Start {}, avoid {} mine {}'.format(start_field, avoidance, mining))

        """
        if not searching and check_crossed(image, yellow_min, yellow_max, 'yellow line') and not load_images:

            print('crossed a yellow line')
            if start_field:
                start_field = False
                avoidance = True
                changedState = True
            elif avoidance:
                avoidance = False
                start_field=True
                changedState = True
            else:
                print('not in any state, there"s a problem with yellow!')
            print('Start {}, avoid {} mine {}'.format(start_field, avoidance, mining))
        '''
        pink_line_min, pink_line_max = (164, 65, 252), (166, 75, 255)
        pink_line_min, pink_line_max = (155, 30, 250), (166, 40, 255)
        # ON the papers
        # pink_line_min, pink_line_max = (155, 115, 170), (179, 150, 199)
        '''
        if not searching and check_crossed(image, pink_line_min, pink_line_max, 'pink line') and not load_images:
            print('Crossed a pink line')
            if avoidance:
                avoidance = False
                mining = True
                changedState = True
            elif mining:
                mining = False
                avoidance = True
                changedState = True
            else:
                print('not in any state, there"s a problem with pink!')

            print('Start {}, avoid {} mine {}'.format(start_field, avoidance, mining))
        """


        if start_field:
            small_img = image.copy()
            small_img = cv.resize(small_img, (width, int(height*0.6)))
            x_mid = width / 2
            y_mid = height / 2
            #cv.imshow("Small Orig", small_img)
            hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
            hsv_img = prepareImage(image)
            filtered_img = cv.inRange(hsv_img, green_ice_min, green_ice_max)
            edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)
            edges = cv.blur(edges, (7,7))
            contours, cnt_count = find_contours("green", image)
            cv.drawContours(image, contours, -1, (0,0,255), 3)

            servo.setTarget(HEADTILT, 1510)

            if changedState:
                # headTilt = min_head_tilt
                # talk
                speaking = ['Crossed into starting field', 'boing boing']
                if talking:
                    talk(speaking)
                print(speaking)
                time.sleep(.1)
                stop()
                changedState = False
                searching = True
                search()

            if hasBall:
                cv.imshow("Small Image", small_img)
                # drop the ball in the cup
                if go_to_bucket:
                    print("GOING TO  BUCKET!")
                    raw_img = hsv_img.copy()
                    resized = cv.resize(raw_img, (width, int(height*0.6)))
                    small_conts, small_cnt_count = find_contours("green", resized)
                    cv.drawContours(small_img, small_conts, -1, (0,0,255),3)

                    if small_cnt_count > 0:
                        rect = cv.boundingRect(small_conts[0])
                        x,y,w,h = rect
                        cv.rectangle(resized, (x,y), (x+w, y+h),(0,0,255))
                        bucket_center = get_rect_center(rect)
                        print("BUCKET CENTER: ", bucket_center)
                        print("SMALL CONTOUR COUNT: ", small_cnt_count)
                        go_straight()
                    elif small_cnt_count == 0:
                        print("CONTOUR COUNT 0")
                        repositioning = False
                        go_to_bucket = False
                        blob_found = False
                        searching = False
                        servo.setTarget(MOTORS, 6000)
                        time.sleep(.5)
                        servo.setTarget(TWIST, twist_in)
                        time.sleep(.5)
                        servo.setTarget(HAND, open_hand)
                        time.sleep(1.0)
                        stop()

                    #else:
                     #   go_straight()
                elif repositioning:
                    # Reposition
                    print("FOUND THE BUCKET! WILL NOW REPOSITION!")
                    searching = False
                    servo.setTarget(HEADTURN, 6000)
                    if cnt_count > 0:
                        rect = cv.boundingRect(contours[0])
                        x,y,w,h = rect
                        cv.rectangle(image, (x,y), (x+w, y+h),(0,0,255))
                        bucket_center = get_rect_center(rect)
                        if bucket_center[0] > x_mid + 50:
                            print("reposition left")
                            reposition("left")
                        elif bucket_center[0] < x_mid - 50:
                            print("reposition right")
                            reposition("right")
                        elif bucket_center[0] >= x_mid - 50 and bucket_center[0] <= x_mid + 50:
                            repositioning = False
                            go_to_bucket = True
                    else:
                        repositioning = False
                        searching = True

                elif cnt_count > 0:
                    print("Found the bucket!")
                    blob_found = True
                    repositioning = True
                else:
                    blob_found = False
                    go_to_bucket = False

            elif not hasBall:
                # go to avoidance to get the ball
                pass

        elif avoidance:
            servo.setTarget(HEADTILT, 1510)
            if changedState:
                # talk
                speaking = ["Vroom vroom. Let's run stuff over!"]
                if talking:
                    talk(speaking)
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
                speaking = ['Time to collect some ice']
                if talking:
                    talk(speaking)
                print(speaking)
                changedState = False
                time.sleep(.15)
                stop()
                headTilt = 6000
                servo.setTarget(HEADTILT, headTilt)
                servo.setTarget(TWIST, twist_in)

            if hasBall:
                if not searchingTurn and not locatedGreenBox:
                    time.sleep(1)
                    servo.setTarget(TWIST, twist_out)
                    #servo.setTarget(HEADTILT, 1510)
                    searchingTurn = True
                    search_turn()
                    print('starting to spin and search for green')
                else:
                    servo.setTarget(HEADTILT, 6000)
                    # locate the green box while searching
                    # green_min, green_max = (20, 160, 120), (75, 225, 190)
                    if not locatedGreenBox:
                        # print('detecting green box!')
                        if detect_green_box(image, green_min, green_max):
                            locatedGreenBox = True
                            searchingTurn = False
                    else:  # locatedGreenBox
                            # reposition until green box is in the center of screen
                        if not moveToGreen:
                            green_turn_dir = which_center_green(image, green_min, green_max)
                            if green_turn_dir == 'center':
                                stop()  # stop for now
                                moveToGreen = True
                                servo.setTarget(HEADTILT, 1510)
                                time.sleep(.25)
                                print('Towards green! go_straight and do magical things!')
                            elif green_turn_dir == 'none':
                                # reset so it starts spinning again
                                locatedGreenBox = False
                                searchingTurn = False
                            else: # left or right
                                reposition(green_turn_dir)
                        else: # moveToGreen
                            go_straight()

            elif not hasBall:


                if gettingIce:
                    servo.setTarget(HEADTILT, 1510)
                    detect_ice(image)
                else:
                    faces = faces_found(image)

                if gettingIce:
                    pass
                elif stopped and not askedForIce:
                    speak = ['I need the ice please']
                    print(speak)
                    if talking:
                        talk(speak)
                        time.sleep(1)
                    askedForIce = True
                    repositioning = False
                    gotoHuman = False
                    gettingIce = True
                elif stopped: # and len(faces) == 0:
                    detect_ice(image)
                    repositioning = False
                    gotoHuman = False
                elif gotoHuman:
                    print("Chase the human!!!!")
                    # The face has been lost too long, stop before people die
                    if face_timer >= doom_timer and not stopped:
                        print('Was chacing, but lost face for too long. Stop!')
                        # print('*************DONE!*********************')
                        stop()
                        # sawHuman = False
                        gotoHuman = False
                        repositioning = False
                        searching = False
                        stopped = True
                    elif len(faces) > 0:
                        face_timer = 0
                        face = faces[0]
                        print('Face {} of screen'.format(str(face[2]/width*100)))
                        # stop if width face is too much
                        if stopped:
                            detect_ice(image)
                            sawHuman = True
                            gotoHuman = False
                            repositioning = False
                            searching = False
                        elif face[2]/width > .15:
                            face_timer = 2
                        elif face[2]/width > .35:
                            print('Was chasing, but got a big face so stop!')
                            stop()
                            # print('*************DONE!*********************')
                            stopped = True
                        else:
                            go_straight()
                    elif stopped:
                        # detect_ice(image)
                        gettingIce = True
                        print('Does the program even reach this stopped option?')

                    else:
                        go_straight()

                elif repositioning:
                    print("repositioning...")
                    sawHuman = True
                    reposition(turn_dir)
                    # stop if face is too close
                    if len(faces) > 0:
                        face_timer = 0
                        print('found a face!')
                        stop_face_center(width, faces)

                # the first time the face found
                elif len(faces) > 0 and not sawHuman:
                    print('greet the human!!!!')
                    repositioning = True
                    # reposition_timer = 0
                    # threading.Timer(1, time_reposition).start()
                    sawHuman = True
                    threading.Timer(0, talk).start()
                    get_turn_dir()
                    headTilt, headTurn = 6000, 6000
                    servo.setTarget(HEADTURN, headTurn)
                    servo.setTarget(HEADTILT, headTilt)
                    reposition(turn_dir)
                    face_timer = 0
                    threading.Timer(1, time_the_faces).start()
                    # cv.imwrite('frame' + str(frame_itter) + '.png', image)

                # The face has been found before
                elif len(faces) > 0:
                    sawHuman = True
                    face_timer = 0
                    print('found a face!')

                elif not sawHuman and not repositioning and not gotoHuman and not searching:
                    searching = True
                    threading.Timer(move_wait_time, search).start()

                elif sawHuman:
                    # The time has run out and there are no faces
                    sawHuman = False
                    # assume the robot is chacing the human or doing something, stop it
                    stop()
                    gotoHuman = False
                    # threading.Timer(move_wait_time, search).start()
                else:
                    # do nothing here, keeps going as normal, no face
                    # ..........
                    sawHuman = False
                    gotoHuman = False
                    # threading.Timer(move_wait_time, search).start()

                # TODO: Does this go in the big conditional or will it work out here?
                # I'm trying it here, because I'm lazy
                if sawHuman:
                    searching = False
                # draw rectangle around the face
                for (x,y,w,h) in faces:
                    cv.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

                cv.imshow("faces", image)


        else:
            print('Error: not a valid state!')
        key = cv.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("c"):
            cv.imwrite('image.png', image)
        if key == ord("q"):
            stop()
            END_PROGRAM = True
            break
        if key == ord('p'):
            pause()

except KeyboardInterrupt:
    print('Manually stopped')
    END_PROGRAM = True
    arm_down()
    stop()

except Exception as e: # catch *all* exceptions
    stop()
    END_PROGRAM = True
    print(traceback.print_tb(e.__traceback__))
    print('Stopped motors due to an error')
    arm_down()

except:
    e = sys.exc_info()
    print(e)
    print('Something didn"t catch....')
    END_PROGRAM = True
    stop()
    arm_down()
    #print(traceback.print_tb(e.__traceback__))
