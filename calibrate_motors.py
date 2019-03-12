import time
import cv2 as cv
import numpy as np
import maestro

MOTORS = 1
TURN = 2

motors = 6000
turn = 6000

min_move = 6100
max_move = 5400
max_turn = 6600  # right
min_turn = 5400  # left

servo = maestro.Controller()

def start_motors():
	global motors
	while motors > max_move - 200:
	    motors -= 300
	    servo.setTarget(MOTORS, motors)
	    time.sleep(0.01)

def turn_left():
	global motors, turn
	print('left')
	while turn > min_turn:
		turn -= 200
		servo.setTarget(TURN, turn)

def turn_right():
	global motors, turn
	print('left')
	while turn < max_turn:
		turn += 200
		servo.setTarget(TURN, turn)


def go_straight():
	global motors, turn
	servo.setTarget(TURN, 6000)
	servo.setTarget(MOTORS, max_move)


def slow_down():
	global motors
	while motors > min_move:
		motors -= 200

def stop():
	servo.setTarget(TURN, 6000)
	servo.setTarget(MOTORS, 6000)


try:
	time.sleep(1)
	start_motors()
	time.sleep(2)
	slow_down()
	turn_left()
	turn_left()
	go_straight()

except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    stop()
    print('Stopped motors due to an error')






