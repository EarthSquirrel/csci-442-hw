import time
import cv2 as cv
import numpy as np

MOTORS = 1
TURN = 2

motors = 6000
turn = 6000

min_move = 6100
max_move = 5400
max_turn = 6600  # right
min_turn = 5400  # left

servo = maestro.controller()

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



def slow_down():
	global motors
	while motors > min_move:
		motors -= 200

def stop():
    servo.setTarget(TURN, 6000)
    servo.setTarget(MOTORS, 6000)





