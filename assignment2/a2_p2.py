import cv2
import numpy as np

cap = cv2.VideoCapture(0)
status, img = cap.read()
prev_frame = None

while(True):
	status, img = cap.read()
	avg = np.float32(img)
	# Create copy of original frame for contours
	tours = img.copy()
	cv2.namedWindow('image1', cv2.WINDOW_NORMAL)
	cv2.namedWindow('avg', cv2.WINDOW_NORMAL)
	cv2.namedWindow('contours', cv2.WINDOW_NORMAL)

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
	# absDiff = cv2.cvtColor(absDiff, cv2.COLOR_BGR2GRAY)
	__,new_image = cv2.threshold(absDiff, 15, 255, cv2.THRESH_BINARY)
	# new_image = cv2.blur(new_image, (5,5))
	# __,new_image = cv2.threshold(new_image, 200, 255, cv2.THRESH_BINARY)

	# DRAW CONTOURS
	contour_im, contours, hierarchy = cv2.findContours(new_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE) 
	for cnt in contours:
		x,y,w,h = cv2.boundingRect(cnt)
		cv2.rectangle(tours, (x,y), (x+w,y+h,), (0,255,0), 2)

	cv2.imshow('image1', new_image)
	cv2.imshow('avg', avg)
	cv2.imshow('contours', tours)

	prev_frame = img

	k = cv2.waitKey(1)
	if k == 27:
		break

cv2.destroyAllWindows()