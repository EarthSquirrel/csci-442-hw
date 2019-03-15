# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import sys
import maestro

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 5 # 5 # 15  #   32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3

motors = 6000
turn = 6000

max_move = 5000  # 5400
max_turn = 6600  # right
min_turn = 5400  # left


# create maeskjslk thing
servo = maestro.Controller()

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h

# control the motors
def head_down():
    pass

def start_motors():
    global motors
    while motors > max_move - 200:
        motors -= 300
        servo.setTarget(MOTORS, motors)
        time.sleep(0.01)


def go_straight():
    global motors, turn, max_move
    print('straight')
    servo.setTarget(TURN, 6000)
    # Do we have to build up to this?
    """
    while motors > max_move:
        motors -= 300
        servo.setTarget(MOTORS, motors)
        time.sleep(0.01)
    """
    motors = max_move
    servo.setTarget(MOTORS, motors)

def turn_right():
    global motors, turn
    print('right')
    # decrease stright? just by a little? so it still has momentum?
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    """
    while turn < max_turn:
        turn += 400
        servo.setTarget(TURN, turn)
        time.sleep(0.01)
    """
    turn = max_turn
    servo.setTarget(TURN, turn)

def turn_left():
    global motors, turn
    print('left')
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    """
    while turn > min_turn:
        print('left: {}'.format(turn))
        turn -= 400
        servo.setTarget(TURN, turn)
        time.sleep(0.01)
    """
    turn = min_turn
    servo.setTarget(TURN, turn)

def stop():
    print('stop')
    servo.setTarget(TURN, 6000)
    servo.setTarget(MOTORS, 6000)




started = False
paused =  False
prev_cog = (0, 0)
cog = (0, 0)

try:
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if started:
            prev_cog = cog
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

        # robot lab settings
        hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
        # cs lab
        # hsv_min, hsv_max = (0, 0, 90), (75, 250, 255)
        color_filter = cv.inRange(hsv, hsv_min, hsv_max)
        # pic = cv.Canny(hsv, 150, 170)

        cv.imshow('hsv',color_filter) # hsv)

        edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
        dil_edges = cv.dilate(edges, kernel, iterations=1)
        cv.imshow('edges', dil_edges)

        contours, hierarchy = cv.findContours(dil_edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
        width, height, channel = img.shape
        tours = 255 * np.ones((width,height,1), np.uint8)
        thresh = img.copy()

        cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
        # print(len(contours))
        # contours = cntsSorted[-3]

        contoursS = sorted(contours, key=lambda x: myContourArea(x))
        contoursS.reverse()
        # if myContourArea(contoursS[-1]) == 0:
        #    del contourssS[-1]
        print(len(contoursS))
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
            # print(myContourArea(cnt))
            if M['m00'] == 0:
                div_by =0.1
            else:
                div_by = M['m00']

            cx = int(M['m10']/div_by)
            cy = int(M['m01']/div_by)
            break
        # print('({}, {})'.format(cx, cy))

        cog = (cx, cy)
        cv.rectangle(thresh, (cx,cy), (cx+29, cy+20),(0,0,255), 2)

        if started  and not paused:
            angle_cut = 0.01 # .02
            angle = np.math.atan2(np.linalg.det([cog,prev_cog]),np.dot(cog,prev_cog))
            # print(angle)  # np.degrees(angle))
            print(angle)
            if angle > angle_cut:
                turn_right()
            elif angle < -angle_cut:
                turn_left()
            else:
                go_straight()

        elif not started and not paused:
            start_motors()

        elif paused:
            pass
        else:
            print('there is an options missing here!!!')

        cv.drawContours(tours, contours, -1, (0,0,255), -1)
        cv.imshow('Contours', thresh)

        started = True


        # show the frame
        # cv.imshow("Frame", pic)
        key = cv.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            stop()
            break
        if key == ord("p"):
            paused = not paused
            print('paused is now: ', paused)
            print('Stopping motors so can look at the vision stuff! :) ')
            stop()

except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    stop()
    print('Stopped motors due to an error')
