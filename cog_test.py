# import the necessary packages
import cv2 as cv
import numpy as np
import sys

print(sys.argv[1])
img = cv.imread(sys.argv[1], cv.IMREAD_COLOR)

cv.namedWindow('Contours', cv.WINDOW_NORMAL)

def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h

width, height, channel = img.shape
img = img[int(1*height/3):height, 20:width-20]
# __,new_image = cv.threshold(img, 0, 400, cv.THRESH_BINARY)
# cv.imshow("newimg", new_image)
blur = cv.blur(img,(7,7))

kernel = np.ones((10,10), np.uint8)

img_erosion = cv.erode(blur, kernel, iterations=2)
img_dilation = cv.dilate(img_erosion, kernel, iterations=3)
# color filtering stuff, save for later
hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)
# hsv = img_dilation.copy()

hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
color_filter = cv.inRange(hsv, hsv_min, hsv_max)
# pic = cv.Canny(hsv, 150, 170)

# cv.imshow('hsv',color_filter) # hsv)

edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
# cv.imshow('edges', edges)

contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
width, height, channel = img.shape
tours = 255 * np.ones((width,height,1), np.uint8)
thresh = img.copy()

contoursS = sorted(contours, key=lambda x: contourArea(x))
# print(len(contours))
# contours = cntsSorted[-3]
contoursS.reverse()
if contourArea(contoursS[-1]) == 0:
    countorsS = contoursS[:-2]

print(contourArea(contoursS[-1]))
if contourArea(contoursS[-1]) == 1:
    del contoursS[-1]
    # contoursS.remove(contoursS[-1]) #  = contoursS[:-2]
    print('removing the value 1....')
print(contourArea(contoursS[-1]))

# contoursS = contoursS[:-2]

for cnt in contoursS:
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(thresh, (x,y), (x+w,y+h), (0,255,0), 2)
    print(contourArea(cnt))

M = cv.moments(cnt)
# print(M)
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
print('({}, {})'.format(cx, cy))
print(contourArea(cnt))
cv.rectangle(thresh,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)

cv.drawContours(tours, contours, -1, (0,0,255), -1)
cv.imshow('Contours', thresh)




cv.waitKey(0)