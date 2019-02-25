import cv2 as cv
import numpy as np
from sys import platform as sys_pf
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

# My code
img = cv.imread('./10_color_reduc.jpg', cv.IMREAD_COLOR)
img_gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
blur = cv.blur(img,(3,3))
edges = cv.Canny(blur,10,300)
ret, thresh = cv.threshold(edges,0,255,0)
img2, contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(edges, contours, -1, (0,255,0), 3)

# Britney's code
# img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)
# img = cv.imread('./imagesWOvideo/two.jpg', cv.IMREAD_COLOR)
# thresh = img.copy()



# Set up parameters for blob detection
params = cv.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 2
params.maxThreshold = 500
params.filterByCircularity = True
params.minCircularity = 0.5
params.maxCircularity = 1
blur = cv.blur(img,(3,3))
# cv.imshow('blur', blur)

kernel = np.ones((4,4), np.uint8)

img_erosion = cv.erode(blur, kernel, iterations=1)
kernel = np.ones((4,4), np.uint8)
img_dilation = cv.dilate(img_erosion, kernel, iterations=1)

cv.imshow('Erosion', img_erosion)
cv.imshow('tracking', img_dilation)

edges = cv.Canny(img_dilation, 10, 300)
cv.imshow('edge', edges)
thresh = edges.copy()
img2,contours,hierarchy = cv.findContours(edges, 1, 2)

# cnt = contours[0]
# M = cv.moments(cnt)
# print(M)
# cv.drawContours(edges, contours, -1, (20,20,20), -1)
# cv.imshow('edge2contour', edges)
# for cnt in contours:
#     x,y,w,h = cv.boundingRect(cnt)
#     cv.rectangle(thresh, (x,y), (x+w,y+h,), (255,255,0), 2)
# # cv.imshow('cont', thresh)

kernel = np.ones((3,3), np.uint8)
edge_dilat= cv.dilate(edges, kernel, iterations=2)
cv.imshow('edge-dialtion', edge_dilat)

"""
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()
"""


detector = cv.SimpleBlobDetector_create(params)


keypoints = detector.detect(edge_dilat)
keypoints_im = cv.drawKeypoints(edge_dilat, keypoints, np.array([]), (0, 255, 0),
                                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow('Keypoints', keypoints_im)
cv.imshow('Edges', edges)
cv.imshow('Original', img)


cv.waitKey(0)
