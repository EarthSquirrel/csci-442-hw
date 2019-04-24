import numpy as np
import cv2 as cv
from picamera.array import PiRGBArray
from picamera import PiCamera
import maestro

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))

max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5


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
        threading.Timer(move_wait_time, search).start()
    else: # no longer searching if found a face
        searching = False

try:
frame_iter = 0
blob_found = False
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        frame_iter += 1
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        width, height, channel = image.shape


except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    stop()
    print('Stopped motors due to an error')
    face_timer = -10



