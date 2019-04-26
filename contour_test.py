import numpy as np
import cv2 as cv
import maestro
import sys
import threading
import time

cap = cv.VideoCapture(0)

#green_ice_min = np.array([30, 180, 140])
#green_ice_max = np.array([75, 225, 190])
green_ice_min = np.array([20, 160, 120])
green_ice_max = np.array([75, 225, 190])

def get_center(cnt):
    x,y,w,h = cv.boundingRect(cnt)
    center = np.array([x+(w/2), y+(h/2)])
    return center

def get_rect_center(rect):
	x,y,w,h = rect
	return np.array([(x+(w/2)), (y+(h/2))])


def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def prepare_image(img):
	kernel = np.ones((10,10), np.uint8)
	blur = cv.blur(img,(7,7))
	erosion = cv.erode(blur, kernel, iterations=4)
	dilation = cv.dilate(erosion, kernel, iterations=2)
	return dilation


kernel = np.ones((10,10), np.uint8)


while(True):
	ret, frame = cap.read()
	height, width, __ = frame.shape
	frame = cv.resize(frame, (int(width*.75), int(height*.75)))
	hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
	hsv = prepare_image(hsv)
	thresh = cv.inRange(hsv, green_ice_min, green_ice_max)
	thresh = prepare_image(thresh)
	edges = cv.Canny(thresh, 35, 150, L2gradient=True)
	edges = cv.blur(edges, (7,7))

	cnt_img, contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
	cv.drawContours(frame, contours, -1, (0,0,255), 3)
	contours_srted = sorted(contours, key=lambda x: contourArea(x))
	if len(contours_srted) > 0:
		cnt = contours_srted[0]
		rect = cv.boundingRect(cnt)
		x,y,w,h = rect
		cv.rectangle(frame, (x,y),(x+w,y+h), (0,0,255),2)
		center = get_rect_center(rect)
		print("CENTER: ", center)
		cv.circle(frame, (int(center[0]), int(center[1])), 20, (0,0,255), -1)

	cv.imshow('frame', frame)
	cv.imshow('thresh', thresh)
	cv.imshow('edges', edges)


	k = cv.waitKey(1)
	if k == 27:
		break