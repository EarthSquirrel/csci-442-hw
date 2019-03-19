from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2 as cv
import maestro
import sys
import time

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

max_move = 5000  # 5400
max_turn = 6600  # left
min_turn = 5000  # right


current_head_pos = 6000

def go_straight():
    global motors, turn, max_move
    # print('straight')
    servo.setTarget(TURN, 6000)
    motors = max_move
    servo.setTarget(MOTORS, motors)

# greater than 6000
def turn_left():
    global motors, turn
    # print('right')
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    # if under it's going right, so switch to left
    if turn < 6000:
        turn = 6000
    turn += 200
    if turn > max_turn:
        turn = max_turn
    servo.setTarget(TURN, turn)

# less than 6000
def turn_right():
# def turn_left(): # Is this a problem????
    global motors, turn
    # print('left')
    servo.setTarget(MOTORS, 6000)  # max_move-400)
    if turn > 6000:
        turn = 6000
    # increment the turn
    turn -= 200
    # check if it's less than min turn, then set to min turn
    if turn < min_turn:
        turn = min_turn
    servo.setTarget(TURN, turn)



def stop():
    print('stop')
    servo.setTarget(TURN, 6000)
    servo.setTarget(MOTORS, 6000)


try:
    # Turn the head to a certain pos
    # servo.setTarget(HEADTURN, 4000)

    current_head_pos = 4000
    servo.setTarget(TURN, 5000)
    time.sleep(2)
    while(True):
        servo.setTarget(TURN, 5800)
        # Turn the robot until the body matches the head pos
        #turn -= 200
        #servo.setTarget(TURN, turn)
        #current_head_pos += 200
        #servo.setTarget(HEADTURN, current_head_pos)

except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    print("it stopped.")
    # keys.arrow (65)
    stop()
    print('Stopped motors due to an error')

