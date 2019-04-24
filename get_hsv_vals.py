import numpy as np
import cv2 as cv

cv.namedWindow("original", cv.WINDOW_NORMAL)
cv.namedWindow("hsv", cv.WINDOW_NORMAL)
cv.namedWindow("filter", cv.WINDOW_NORMAL)

img = cv.imread('image-.png')
hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)


#BGR - BAD!
#yellow_min = np.array([110,200,235])
#yellow_max = np.array([135, 215, 246])

#HSV - GOOD!
yellow_min = np.array([20, 110, 235])
yellow_max = np.array([23, 130, 245])

white_min = np.array([100, 5, 230])
white_max = np.array([130, 20, 255])

pink_min = np.array([164, 65, 252])
pink_max = np.array([166, 75, 255])

hsv_filter = cv.inRange(hsv_img, pink_min, pink_max)


def get_hsv_val(event, x, y, flags, param):
    global hsv_img
    if event == cv.EVENT_LBUTTONDOWN:
        img2 = hsv_img
        vals = img2[y][x]
        print("Vals: ", vals)


cv.setMouseCallback("hsv", get_hsv_val)

while True:
	cv.imshow('original', img)
	cv.imshow('hsv', hsv_img)
	cv.imshow('filter', hsv_filter)
	
	k = cv.waitKey(1)
	if k == 27:
		break