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

END_PROGRAM = False # use this to kill the threads
increasing = False

MOTORS = 2
TURN = 1
BODY = 0
HEADTILT = 4
HEADTURN = 3
ARM = 6
HAND = 11
TWISTARM = 7

servo.setTarget(HEADTILT, 5000)

max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000


yellow_min = np.array([20, 110, 235])
yellow_max = np.array([23, 130, 245])

white_min = np.array([100, 5, 230])
white_max = np.array([130, 20, 255])

yellow_ice_min = np.array([30, 135, 200])
yellow_ice_max = np.array([35, 170, 220])

#green_ice_min = np.array([30, 180, 140])
#green_ice_max = np.array([75, 225, 190])
green_ice_min = np.array([20, 160, 120])
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
                # maxed out array, return to o
    # print("\t\t\tSearching: Head Pos: ", headTurn)
    servo.setTarget(HEADTURN, headTurn)

    if not blob_found:
        threading.Timer(1.0, search).start()
    else: # no longer searching if found a face
        searching = False


def reposition(turn_dir):
    global turn
    if turn_dir == 'right':
        print('\treposition to the right')
        # turn -= 200
        turn = min_turn
        if turn < min_turn:
            turn = min_turn
    elif turn_dir == 'left':
        print('\treposition to the left')
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

def pause():
    # print('in stop: stopping motors')
    global motors, turn, body, headTurn, headTilt
    motors = 6000
    turn = 6000
    body = 6000


def go_forward():
    servo.setTarget(MOTORS, 5400)


def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def prepareImage(img):
    kernel = np.ones((10,10), np.uint8)
    blur = cv.blur(img,(7,7))
    erosion = cv.erode(blur, kernel, iterations=4)
    dilation = cv.dilate(erosion, kernel, iterations=2)
    return dilation


def get_center(cnt):
    x,y,w,h = cv.boundingRect(cnt)
    center = np.array([x+(w/2), y+(h/2)])
    return center


def get_rect_center(rect):
    x,y,w,h = rect
    return np.array([(x+(w/2)), (y+(h/2))])


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


def find_blob(color, img):
    prep_img = prepareImage(img)
    hsv_img = cv.cvtColor(prep_img, cv.COLOR_BGR2HSV)
    detector = cv.SimpleBlobDetector_create()

    if color == "green":
        min_hsv = green_ice_min
        max_hsv = green_ice_max

    filtered_img = cv.inRange(hsv_img, min_hsv, max_hsv)
    edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)


ice_min = np.array([20,20,20])
ice_max = np.array([255,255,255])

try:
    servo.setTarget(HEADTILT, 3000)
    blob_found = False
    go_to_bucket = False
    repositioning = False
    search()
# capture frames from the camera
    time.sleep(0.1)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        small_img = image.copy()
        height, width, channel = image.shape
        x_mid = width / 2
        y_mid = height / 2
        small_img = cv.resize(small_img, (width, int(height*0.6)))
        cv.imshow("Small Orig", small_img)
        hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hsv_img = prepareImage(image)
        filtered_img = cv.inRange(hsv_img, green_ice_min, green_ice_max)
        edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)
        edges = cv.blur(edges, (7,7))
        contours, cnt_count = find_contours("green", image)


        if go_to_bucket:
            print("Going to bucket!")
            raw_img = hsv_img.copy()
            resized = cv.resize(raw_img, (width, int(height*0.6)))
            small_conts, small_cnt_count = find_contours("green", resized)
            rect = cv.boundingRect(contours[0])
            x,y,w,h = rect
            cv.rectangle(resized, (x,y), (x+w, y+h),(0,0,255))
            bucket_center = get_rect_center(rect)
            print("BUCKET CENTER: ", bucket_center)
            print("SMALL CONTOUR COUNT: ", small_cnt_count)
            '''
            if bucket_center[1] < 260:
                go_to_bucket = False
                stop()
            else:
                go_forward()
            '''
            go_forward()
        elif repositioning:
            # Reposition
            print("FOUND THE BUCKET! WILL NOW REPOSITION!")
            servo.setTarget(HEADTURN, headTurn)
            if cnt_count > 0:
                rect = cv.boundingRect(contours[0])
                x,y,w,h = rect
                cv.rectangle(image, (x,y), (x+w, y+h),(0,0,255))
                bucket_center = get_rect_center(rect)
                if bucket_center[0] > x_mid + 50:
                    reposition("right")
                elif bucket_center[0] < x_mid - 50:
                    reposition("left")
                elif bucket_center[0] >= x_mid - 50 and bucket_center[0] <= x_mid + 50:
                    print("GOING TO BUCKET")
                    repositioning = False
                    go_to_bucket = True

        elif cnt_count > 0:
            print("Found the bucket!")
            blob_found = True
            repositioning = True
        else:
            blob_found = False
            go_to_bucket = False

        key = cv.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break


        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)



except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    stop()
    print('Stopped motors due to an error')
    face_timer = -10



