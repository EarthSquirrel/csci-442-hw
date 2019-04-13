# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import maestro
import sys
import threading
from client import ClientSocket

# Set everything to 0 to start
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


def time_reposition():
    if END_PROGRAM:
        return
    global reposition_timer
    reposition_timer += 1
    print('\t\treposition_timer: ', reposition_timer)
    if repositioning:
        threading.Timer(1,time_reposition).start()

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
            tilt_loc += 1
            if tilt_loc > 2:
                # maxed out array, return to o
                tilt_loc= 0
            headTilt = tilt_positions[tilt_loc]
    # print("\t\t\tSearching: Head Pos: ", headTurn)
    servo.setTarget(HEADTURN, headTurn)
    servo.setTarget(HEADTILT, headTilt)

    if not face_found:
        threading.Timer(move_wait_time, search).start()
    else: # no longer searching if found a face
        searching = False



def talk(say_this):
    IP = '10.200.2.215'
    PORT = 5010
    client = ClientSocket(IP, PORT)
    ##client.start()
    for i in say_this: # ["hello human", "How are you", "Sorry, you must die now"]:
        client.sendData(i)
        print('\tspeaking: ', i)
        time.sleep(1)
    print("\tExiting Sends")
    client.killSocket()

def reposition(turn_dir, head_pos, frame):
    global turn
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
        print('Going streight')
        # print('\tYa, something broke....')
    servo.setTarget(TURN, turn)
    time.sleep(0.25)
    # print('\trepositioning turn value: ' + str(servo.getPosition(TURN)))
    turn = 6000
    servo.setTarget(TURN, turn)
    time.sleep(.5)

def go_forward():
    servo.setTarget(MOTORS, max_move)

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


##################################################3##
############## Boolean values #######################
#####################################################

# states
start_field = True
avoidance = False
mining = False

# global accross all states
hasBall = False

# start_filed bools
droppedBall = False

# mining bools
sawHuman = False
getBall = False


# allow the camera to warmup
time.sleep(0.1)

try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        width, height, channel = image.shape

        if start_field:

        elif avoidance:

        elif mining:

        else:
            print('Error: not a valid state!')



except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    stop()
    print('Stopped motors due to an error')
    face_timer = -10
