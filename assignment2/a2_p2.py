import cv2
import numpy as np

cap = cv2.VideoCapture(0)
status, img = cap.read()
prev_frame = None

while(True):
	status, img = cap.read()
	avg = np.float32(img)
	# Create copy of original frame for thresh
	thresh = img.copy()
	# Create white image for contours
	width, height, channel = img.shape
	tours = 255 * np.ones((width,height,1), np.uint8)
	cv2.namedWindow('Tours', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Contours', cv2.WINDOW_NORMAL)
	cv2.namedWindow('Thresh', cv2.WINDOW_NORMAL)

	avg = cv2.cvtColor(avg, cv2.COLOR_BGR2GRAY)
	img = cv2.cvtColor(np.float32(img), cv2.COLOR_BGR2GRAY)
	avg = cv2.blur(avg, (15,15))
	img = cv2.blur(img, (15,15))

	if prev_frame is None:
		prev_frame = img.copy().astype(float)
		continue
	
	cv2.accumulateWeighted(cv2.convertScaleAbs(prev_frame), avg, 0.001)
	avg = cv2.convertScaleAbs(avg)
	img = cv2.convertScaleAbs(img)
	absDiff = cv2.absdiff(cv2.convertScaleAbs(prev_frame),img)
	__,new_image = cv2.threshold(absDiff, 50, 255, cv2.THRESH_BINARY)
	new_image = cv2.blur(new_image, (5,5))

	# DRAW CONTOURS
	contour_im, contours, hierarchy = cv2.findContours(new_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE) 
	for cnt in contours:
		x,y,w,h = cv2.boundingRect(cnt)
		cv2.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)

	cv2.drawContours(tours, contours, -1, (0,0,0), -1)
	cv2.imshow('Thresh', new_image)
	cv2.imshow('Contours', thresh)
	cv2.imshow('Tours', tours)

	prev_frame = img

	k = cv2.waitKey(1)
	if k == 27:
		break

cv2.destroyAllWindows()