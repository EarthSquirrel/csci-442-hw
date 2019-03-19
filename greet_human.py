# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import maestro
import sys
import threading

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640, 480))
face_cascade = cv.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('haarcascade_eye.xml')

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


stop()

max_turn = 7500 # max 7900
min_turn = 4000 # min 1510
max_time = 5

tilt_positions = [5000, 6000, 7000]
tilt_loc = 0  # location in positions
increasing = True  # tell what direction it's going
eof = False  # Move every other frame
face_timer = max_time + 1


def time_the_faces():
    global face_timer
    face_timer += 1
    # print('\t\tface_timer: ', face_timer)
    threading.Timer(1, time_the_faces).start()


time_the_faces()
face_found = False
def faces_found(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)
    real_faces = []
    # if len(faces) > 0:
        # print('Faces total found: {}'.format(len(faces)))
    for (x,y,w,h) in faces:
        roi_gray = gray[y:y+h, x:x+w]
        eyes = eye_cascade.detectMultiScale(roi_gray)
        # If there are no eyes, don't use this face
        if len(eyes) == 0:
            continue
        #Otherwise, we use it and return it to our list of return faces
        # print('total eye count: {}'.format(len(eyes)))
        real_faces.append((x,y,w,h))
        # cv.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),2)
    return real_faces

# allow the camera to warmup
time.sleep(0.1)

try:
# capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array

        # TODO: Scanning code here!
        # use tild position and increment
        if not face_found:
            if increasing:
                headTurn += 100
                if headTurn > max_turn:
                    headTurn = max_turn
                    increasing = False
            else:
                headTurn -= 100
                if headTurn < min_turn:
                    headTurn = min_turn
                    increasing = True
                    tilt_loc += 1
                    if tilt_loc > 2:
                        # maxed out array, return to o
                        tilt_loc= 0
                    headTilt = tilt_positions[tilt_loc]
            servo.setTarget(HEADTURN, headTurn)
            servo.setTarget(HEADTILT, headTilt)

            # check if max or min scan head turn values been found

            # run facial recognition on image
        faces = faces_found(image)
        if len(faces) > 0:
            face_found = True
            face_timer = 0
            print('found a face!')
        elif face_timer < max_time:
            face_found = True
        else:
            # The time has run out and there are no faces
            face_found = False

        # draw rectangle around the face
        for (x,y,w,h) in faces:
            cv.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)

        cv.imshow("Frame", image)
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
    stop()
    print('Stopped motors due to an error')
    sys.exit()
