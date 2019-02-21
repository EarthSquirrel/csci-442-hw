import cv2 as cv
import numpy as np

img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)

# Set up parameters for blob detection
params = cv.SimpleBlobDetector_Params()

# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200
# params.filterByCircularity = True
# params.minCircularity = 0.75
# params.maxCircularity = 1

detector = cv.SimpleBlobDetector_create(params)

keypoints = detector.detect(img)
keypoints_im = cv.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv.imshow('Keypoints', keypoints_im)

print(len(img[0]))

cv.waitKey(0)