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

def check_crossed(raw_img,  hsv_min, hsv_max):
    global old_cog_line
    raw_img = raw_img[int(height/3):height, int(width/5):int(4*width/5)] # int(width*.25):int(width*.75)]


    color_filter = get_hsv_filter(raw_img, hsv_min, hsv_max)
    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    kernel = np.ones((10,10), np.uint8)
    dil_edges = cv.dilate(edges, kernel, iterations=1)
    # cv.imshow('edges', dil_edges)

    thresh = raw_img.copy()

    contoursS = get_contours(dil_edges)

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
    threading.Timer(1,time_the_faces).start()

def get_turn_dir():
    global turn_dir
    head_pos = servo.getPosition(HEADTURN)
    face_loc = faces[0][0] + faces[0][2]/2
    # if (head_pos < 6000 and face_loc < width/2) or (head_pos > 6000 and face_loc < width/2):
    if head_pos > 6000:
        print("Turn dir = right")
        turn_dir = "right"
    # elif (head_pos > 6000 and face_loc > width/2) or (head_pos < 6000 and face_loc > width/2):
    if head_pos < 5000:
        print("Turn dir = left")
        turn_dir = 'left'


def stop_face_size(width, faces):
    face_center = faces[0][0] + 0.5*faces[0][2]
    if abs(face_center - width/2) < width/5:
        print('Face in center, stoping things')
        turn = 6000
        servo.setTarget(TURN, turn)
        repositioning = False
        gotoHuman = True


def reposition(turn_dir):
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

# TODO: Import new method
def detect_ice(raw_img):
    global counting_ice_cnts, counting_ice_cnts_timer, hasBall
    width, height, channel = raw_img.shape
    width_array = len(raw_img[0])
    raw_img = raw_img[int(height/6):int(2*height/3),int(width_array/2):width_array]#int(width/3):width] # int(width*.25):int(width*.75)]


    hsv_min, hsv_max = (25, 0, 240), (35, 170, 255)
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
    elif len(ice_cnts) > 0 and not hasBall:
        print('first time seeing ice cnts')
        counting_ice_cnts = True
        counting_ice_cnts_timer = 0
        threading.Timer(1, cnts_ice_timer).start()

    thresh = raw_img.copy()
    for cnt in ice_cnts:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)
    cv.imshow('ice seen', thresh)



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

def pause():
    print('in stop: stopping motors')
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
ARM = 6
HAND = 10
ELBOW = 8
TWIST = 7
WRIST = 11



motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000

move_arms = False
max_move = 5400  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
min_head_tilt = 1510
max_time = 5

open_hand = 4000
closed_hand = 5600
raised_arm = 8000
lower_arm = 4000
elbow_straight = 6000
twist_in = 5000
wrist = 6000

# set arm so don't have to deal with it later
servo.setTarget(ARM, raised_arm)
servo.setTarget(HAND, open_hand)
servo.setTarget(TWIST, twist_in)
servo.setTarget(WRIST, wrist)


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
time_to_cross_line = 3

# start_filed bools
droppedBall = False
old_cog_line = (float('inf'), float('inf'))

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

#

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
#avoidance = True
#start_field = False

try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        width, height, channel = image.shape
        # yellow
        yellow_min, yellow_max = (0, 40, 90), (75, 250, 255)

        # Change the state if crossed a line
        if not searching and check_crossed(image, yellow_min, yellow_max):
            changedState = True
            if start_field:
                start_field = False
                avoidance = True
            elif avoidance:
                avoidance = False
                start_field=True
            else:
                print('not in any state, there"s a problem with yellow!')

        pink_min, pink_max = (164, 65, 252), (166, 75, 255)
        if not searching and check_crossed(image, pink_min, pink_max):
            if avoidance:
                avoidance = False
                mining = True
            elif mining:
                mining = False
                avoidance = True
            else:
                print('not in any state, there"s a problem with pink!')

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
                stop()
                headTilt = 6000
                servo.setTarget(HEADTILT, headTilt)

            if hasBall:
                print('The robot now has the ice!')

            elif not hasBall:

                faces = faces_found(image)

                if stopped and len(faces) == 0:
                    # needs to wait seven seconds before searching again
                    print('Lost the face, now I will wait 7 seconds...')
                    time.sleep(7)
                    stopped = False
                    sawHuman = False
                    repositioning = False
                    gotoHuman = False
                # TODO: What happens with stop down here?
                elif gotoHuman:
                    print("Chase the human!!!!")
                    # The face has been lost too long, stop before people die
                    if face_timer > doom_timer and not stopped:
                        print('Was chacing, but lost face for too long. Stop!')
                        print('*************DONE!*********************')
                        stop()
                        sawHuman = False
                        gotoHuman = False
                        repositioning = False
                        searching = False
                        stopped = True
                    elif len(faces) > 0:
                        face = faces[0]
                        print('Face {} of screen'.format(str(face[2]/width*100)))
                        # stop if width face is too much
                        if stopped:
                            if face_timer > 7:
                                sawHuman = False
                                gotoHuman = False
                                repositioning = False
                                searching = False
                                stopped = False
                        elif face[2]/width > .17:
                            print('Was chasing, but got a big face so stop!')
                            stop()
                            print('*************DONE!*********************')
                            stopped = True
                        else:
                            go_straight()
                    elif stopped:
                        detect_ice(image)

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
                        stop_face_size(width, faces)

                # the first time the face found
                elif len(faces) > 0 and not sawHuman:
                    print('greet the human!!!!')
                    repositioning = True
                    # reposition_timer = 0
                    # threading.Timer(1, time_reposition).start()
                    sawHuman = True
                    threading.Timer(0, talk).start()
                    turn_dir = get_turn_dir()
                    headTilt, headTurn = 6000, 6000
                    servo.setTarget(HEADTURN, headTurn)
                    servo.setTarget(HEADTILT, headTilt)
                    reposition(turn_dir)
                    face_timer = 0
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


                """
                # almost whole area needs access to faces array
                faces = faces_found(image)

                if not sawHuman:
                    # if searching hasn't started, start
                    if not searching:
                        stop()
                        searching = True
                        headTilt = 6000
                        servo.setTarget(HEADTILT, headTilt)
                        threading.Timer(move_wait_time, search).start()
                    elif searching:
                        search_for_face(image, faces)

                elif sawHuman:
                    # get the ball
                    # bools here, gotoHuman, repositioning
                    # Stop searching if found the humna

                    if searching:
                        # saw human, stop searching, start repositioning
                        print('sawHuman, searching')
                        searching = False
                        repositioning = True
                        headTurn = 6000
                        servo.setTarget(headTurn, HEADTURN)

                    if len(faces) > 0:
                        if repositioning:
                            print('>0, sawHuman, repositioning')
                            # move until finds the human
                            reposition(turn_dir, image)

                        elif gotoHuman:
                            print('>0, sawHuman, gotoHuman')
                            # move to the human
                    else:  # doesn't see a face on screen
                        if repositioning:
                            reposition(turn_dir, image)
                            print("Hopefully I don't go in a circle")
                        elif gotoHuman:
                            # check the timer
                            pass

                    if getBall:
                        print('sawHuman, getBall')


                    chase_human(image)
        """
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
