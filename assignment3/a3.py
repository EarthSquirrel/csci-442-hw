import cv2 as cv
import numpy as np
from sys import platform as sys_pf
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")
from matplotlib import pyplot as plt

img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)
img_gray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
blur = cv.blur(img,(3,3))
edges = cv.Canny(blur,10,300)
ret, thresh = cv.threshold(edges,0,255,0)
img2, contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(edges, contours, -1, (0,255,0), 3)





# Set up parameters for blob detection
# params = cv.SimpleBlobDetector_Params()
# Change thresholds
# params.minThreshold = 10
# params.maxThreshold = 200
# params.filterByCircularity = True
# params.minCircularity = 0.25
# params.maxCircularity = 1
# params.filterByInertia = True
# params.minInertiaRatio = 0.9

# detector = cv.SimpleBlobDetector_create(params)

# keypoints = detector.detect(img)
# keypoints_im = cv.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

# cv.imshow('Keypoints', keypoints_im)
cv.imshow('Edges', edges)
cv.imshow('Original', img)


cv.waitKey(0)