import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)
# img = cv.imread('./imagesWOvideo/two.jpg', cv.IMREAD_COLOR)
thresh = img.copy()

# Set up parameters for blob detection
params = cv.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 2
params.maxThreshold = 500
params.filterByCircularity = True
params.minCircularity = 0.5
params.maxCircularity = 1
blur = cv.blur(img,(3,3))
cv.imshow('blur', blur)

kernel = np.ones((4,4), np.uint8)

img_erosion = cv.erode(blur, kernel, iterations=1)
kernel = np.ones((4,4), np.uint8)
img_dilation = cv.dilate(img_erosion, kernel, iterations=1)

# cv.imshow('Erosion', img_erosion)
# cv.imshow('tracking', img_dilation)

edges = cv.Canny(img_dilation, 10, 300)
# cv.imshow('edge', edges)
thresh = edges.copy()
img2,contours,hierarchy = cv.findContours(edges, 1, 2)


# cnt = contours[0]
# M = cv.moments(cnt)
# print(M)
# cv.drawContours(edges, contours, -1, (20,20,20), -1)
# cv.imshow('edge2contour', edges)
for cnt in contours:
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(thresh, (x,y), (x+w,y+h,), (255,255,0), 2)
# cv.imshow('cont', thresh)

"""
# did better without dilation of edges
kernel = np.ones((3,3), np.uint8)
edge_dilat= cv.dilate(edges, kernel, iterations=1)
cv.imshow('edge-dialtion', edge_dilat)

detector = cv.SimpleBlobDetector_create(params)

keypoints = detector.detect(edge_dilat)
keypoints_im = cv.drawKeypoints(edge_dilat, keypoints, np.array([]), (0, 255, 0),
                                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow('Keypoints-dilat', keypoints_im)
"""

detector = cv.SimpleBlobDetector_create(params)

keypoints = detector.detect(edges)
keypoints_im = cv.drawKeypoints(edges, keypoints, np.array([]), (0, 255, 0),
                                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow('Keypoints', keypoints_im)


# Draw new smaller circles
width, height, channel = img.shape
white_img = 0* np.ones((width, height, 1), np.uint8)
color_img = 0* np.ones((width, height, 1), np.uint8)
color_img = cv.cvtColor(color_img, cv.COLOR_GRAY2RGB)

mms = []
for k in keypoints:
    tmp = 0* np.ones((width, height, 1), np.uint8)
    (x, y) = k.pt
    radius = int(k.size-5)
    cv.circle(white_img, (int(x), int(y)), 0, (255, 255, 255), radius)
    cv.circle(tmp, (int(x), int(y)), 0, (255, 255, 255), radius)
    mms.append([k])
    mc = cv.mean(img,mask = tmp)
    mc = [int(m) for m in mc]
    # print(mc)
    cv.circle(color_img, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)

cv.imshow('White Cirlces', white_img)
cv.imshow('Color circles', color_img)
lab_img = cv.cvtColor(color_img, cv.COLOR_RGB2Lab)
cv.imshow('labels', lab_img)

mean_val = cv.mean(img,mask = white_img)
print(mean_val)
# get contures of white circles
thresh = white_img.copy()
img2,contours,hierarchy = cv.findContours(white_img, 1, 2)


# cnt = contours[0]
# M = cv.moments(cnt)
# print(M)
# cv.drawContours(white_img, contours, -1, (20,20,20), -1)
# cv.imshow('edge2contour', white_img)

for cnt in contours:
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(thresh, (x,y), (x+w,y+h,), (50,100,200), 2)
# cv.imshow('cont', thresh)


cv.waitKey(0)
