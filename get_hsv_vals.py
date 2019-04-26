import numpy as np
import cv2 as cv

cv.namedWindow("original", cv.WINDOW_NORMAL)
cv.namedWindow("hsv", cv.WINDOW_NORMAL)
cv.namedWindow("filter", cv.WINDOW_NORMAL)
cv.namedWindow("hsv ice img", cv.WINDOW_NORMAL)

img = cv.imread('image-.png')# 'hand-imag2e.png')
yellow_img = cv.imread('yellow.png')
green_img = cv.imread('green.png')
hsv_ice_img = cv.cvtColor(yellow_img, cv.COLOR_BGR2HSV)
thresh = green_img

<<<<<<< HEAD
=======
img = cv.imread('paper-field-hall-429.png')# 'hand-imag2e.png')
>>>>>>> d175e6bdcee5f12d30079ba4de3451d6a97bf62f
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

yellow_ice_min = np.array([30, 135, 200])
yellow_ice_max = np.array([35, 170, 220])

<<<<<<< HEAD
green_ice_min = np.array([50, 190, 160])
green_ice_max = np.array([55, 205, 170])

hsv_filter = cv.inRange(hsv_ice_img, yellow_ice_min, yellow_ice_max)

# worked on hand-imag2e.png
# yellow_min = np.array([100, 0, 150])
# yellow_max = np.array([135, 15, 255])
=======
hsv_min, hsv_max = (150, 100, 230), (170, 140, 255)

hsv_filter = cv.inRange(hsv_img, hsv_min, hsv_max)
>>>>>>> d175e6bdcee5f12d30079ba4de3451d6a97bf62f


def get_hsv_val(event, x, y, flags, param):
    global hsv_ice_img
    if event == cv.EVENT_LBUTTONDOWN:
        img2 = hsv_ice_img
        vals = img2[y][x]
        print("Vals: ", vals)


cv.setMouseCallback("hsv ice img", get_hsv_val)

while True:
	cv.imshow('original', green_img)
	cv.imshow('hsv ice img', hsv_ice_img)
	cv.imshow('filter', hsv_filter)

	img2, contours, hierarchy = cv.findContours(hsv_filter, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
	# for cnt in contours:
	# 	x,y,w,h = cv.boundingRect(cnt)
	# 	cv.rectangle(hsv_filter, (x,y), (x+w,y+h,), (0,255,0), 2)

	cv.drawContours(hsv_filter, contours, -1, (0,255,0), -1)

	k = cv.waitKey(1)
	if k == 27:
		break
