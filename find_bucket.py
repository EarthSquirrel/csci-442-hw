import numpy as np
import cv2 as cv
from picamera.array import PiRGBArray
from picamera import PiCamera
import maestro
import sys
import threading
import time

cv.namedWindow("hsv", cv.WINDOW_NORMAL)


camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))
servo = maestro.Controller()

MOTORS = 1
TURN = 2
BODY = 0
HEADTILT = 4
HEADTURN = 3
ARM = 6
HAND = 11
TWISTARM = 7


max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

yellow_min = np.array([20, 110, 235])
yellow_max = np.array([23, 130, 245])

white_min = np.array([100, 5, 230])
white_max = np.array([130, 20, 255])

yellow_ice_min = np.array([30, 135, 200])
yellow_ice_max = np.array([35, 170, 220])

green_ice_min = np.array([30, 180, 140])
green_ice_max = np.array([75, 225, 190])


def get_hsv_val(event, x, y, flags, param):
    global hsv_img
    if event == cv.EVENT_LBUTTONDOWN:
        img2 = hsv_img
        vals = img2[y][x]
        print("Vals: ", vals)


cv.setMouseCallback("hsv", get_hsv_val)



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

    if not blob_found:
        threading.Timer(1.0, search).start()
    else: # no longer searching if found a face
        searching = False

def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def find_blob(color, img):
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    if color == "green":
        min_hsv = green_ice_min
        max_hsv = green_ice_max

    filtered_img = cv.inRange(hsv_img, min_hsv, max_hsv)
    edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)


    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contoursS = sorted(contours, key=lambda x: contourArea(x))
    '''
    cx, cy = 0,0
    for cnt in contoursS:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h), (0,255,0), 2)
        cv.rectangle(edge_copy, (x,y), (x+w,y+h), (0,255,0), 2)
        print(contourArea(cnt))

        M = cv.moments(cnt)
        # print(M)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        print('({}, {})'.format(cx, cy))
        print(contourArea(cnt))
        break
    '''
    return len(contours)

#cv.rectangle(thresh,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)
#cv.rectangle(edge_copy,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)

#cv.drawContours(tours, contours, -1, (0,0,255), -1)
#cv.imshow('Contours', thresh)
#cv.imshow('edge contour', edge_copy)

ice_min = np.array([20,20,20])
ice_max = np.array([255,255,255])

try:
    servo.setTarget(HEADTILT, 5000)
    blob_found = False
# capture frames from the camera
    time.sleep(0.1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        width, height, channel = image.shape
        hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        filtered_img = cv.inRange(hsv_img, green_ice_min, green_ice_max)
        edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)


        print("BLOB COUNT: ", find_blob("green", image))
        cv.imshow("edges", edges)
        cv.imshow("hsv", hsv_img)
        cv.imshow("filtered", filtered_img)
        key = cv.waitKey(1) & 0xFF

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    stop()
    print('Stopped motors due to an error')
    face_timer = -10



