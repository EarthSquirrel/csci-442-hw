import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt


img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)
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

cv.imshow('Erosion', img_erosion)
cv.imshow('tracking', img_dilation)

edges = cv.Canny(img_dilation, 10, 300)
cv.imshow('edge', edges)
thresh = edges.copy()
img2,contours,hierarchy = cv.findContours(edges, 1, 2)

cnt = contours[0]
M = cv.moments(cnt)
print(M)
cv.imshow('contour2', img2)
cv.drawContours(edges, contours, -1, (20,20,20), -1)
cv.imshow('edge2contour', edges)
for cnt in contours:
    x,y,w,h = cv.boundingRect(cnt)
    cv.rectangle(thresh, (x,y), (x+w,y+h,), (255,255,0), 2)
cv.imshow('cont', thresh)
"""
plt.subplot(121),plt.imshow(img,cmap = 'gray')
plt.title('Original Image'), plt.xticks([]), plt.yticks([])
plt.subplot(122),plt.imshow(edges,cmap = 'gray')
plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
plt.show()
"""

detector = cv.SimpleBlobDetector_create(params)

keypoints = detector.detect(edges)
keypoints_im = cv.drawKeypoints(edges, keypoints, np.array([]), (0, 0, 255),
                                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow('Keypoints', keypoints_im)

print(len(img[0]))

cv.waitKey(0)
