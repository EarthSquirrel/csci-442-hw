# import the necessary packages
import cv2 as cv
import numpy as np
import sys

print(sys.argv[1])
img = cv.imread(sys.argv[1], cv.IMREAD_COLOR)

cv.namedWindow('Contours', cv.WINDOW_NORMAL)
cv.namedWindow('edge contour', cv.WINDOW_NORMAL)

cv.namedWindow('filtered', cv.WINDOW_NORMAL)
def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h

width, height, channel = img.shape
img = img[int(1*height/3):height, 20:width-20]
# __,new_image = cv.threshold(img, 0, 400, cv.THRESH_BINARY)
# cv.imshow("newimg", new_image)
blur = cv.blur(img,(7,7))

kernel = np.ones((10,10), np.uint8)

img_erosion = cv.erode(blur, kernel, iterations=4)
img_dilation = cv.dilate(img_erosion, kernel, iterations=2)

# color filtering stuff, save for later
hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)
# hsv = img_dilation.copy()

hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
color_filter = cv.inRange(hsv, hsv_min, hsv_max)
# pic = cv.Canny(hsv, 150, 170)

cv.imshow('filtered',color_filter) # hsv)

edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
dil_edges = edges.copy()
dil_edges = cv.dilate(edges, kernel, iterations=3)
# cv.imshow('edges', edges)

contours, hierarchy = cv.findContours(dil_edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
width, height, channel = img.shape
tours = 255 * np.ones((width,height,1), np.uint8)
thresh = img.copy()
edge_copy = cv.cvtColor(dil_edges.copy(), cv.COLOR_GRAY2BGR)

contoursS = sorted(contours, key=lambda x: contourArea(x))
# print(len(contours))
# contours = cntsSorted[-3]
contoursS.reverse()


# contoursS = contoursS[:-2]

cx, cy = 0,0
for cnt in contoursS:
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(thresh, (x,y), (x+w,y+h), (0,255,0), 2)
    cv.rectangle(edge_copy, (x,y), (x+w,y+h), (0,255,0), 2)
    print(contourArea(cnt))

    M = cv.moments(cnt)
    # print(M)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    print('({}, {})'.format(cx, cy))
    print(contourArea(cnt))
    break
cv.rectangle(thresh,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)
cv.rectangle(edge_copy,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)

cv.drawContours(tours, contours, -1, (0,0,255), -1)
cv.imshow('Contours', thresh)
cv.imshow('edge contour', edge_copy)



cv.waitKey(0)
