import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.namedWindow("Video")
cv2.namedWindow('hsv', cv2.WINDOW_NORMAL)
cv2.namedWindow('tracking', cv2.WINDOW_NORMAL)


def get_hsv_val(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        stat2, img2 = cap.read()
        vals = img2[y][x]
        # set track bar to -10 + 10 of click
        adjust = 25
        cv2.setTrackbarPos('H Min', 'hsv', vals[0]-adjust)
        cv2.setTrackbarPos('H Max', 'hsv', vals[0]+adjust)
        cv2.setTrackbarPos('S Min', 'hsv', vals[1]-adjust)
        cv2.setTrackbarPos('S Max', 'hsv', vals[1]+adjust)
        cv2.setTrackbarPos('V Min', 'hsv', vals[2]-adjust)
        cv2.setTrackbarPos('V Max', 'hsv', vals[2]+adjust)


cv2.setMouseCallback("hsv", get_hsv_val)

def nothing(x):
    pass

cv2.createTrackbar('H Min','hsv',0,255, nothing)
cv2.createTrackbar('H Max','hsv',0,255, nothing)
cv2.createTrackbar('S Min','hsv',0,255, nothing)
cv2.createTrackbar('S Max','hsv',0,255, nothing)
cv2.createTrackbar('V Min','hsv',0,255, nothing)
cv2.createTrackbar('V Max','hsv',0,255, nothing)

while True:
    status, img = cap.read()
    cv2.resizeWindow("Video", 600,400)
    cv2.imshow("Video", img)
    cv2.resizeWindow("hsv", 600,400)
    cv2.resizeWindow("tracking", 600,400)

    # get slider values
    h = cv2.getTrackbarPos('H Max','hsv')
    s = cv2.getTrackbarPos('S Max','hsv')
    v = cv2.getTrackbarPos('V Max','hsv')

    hsv_max = np.array([h, s, v])

    h = cv2.getTrackbarPos('H Min','hsv')
    s = cv2.getTrackbarPos('S Min','hsv')
    v = cv2.getTrackbarPos('V Min','hsv')

    hsv_min = np.array([h, s, v])

    # Convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow('hsv', hsv)

    # Color tracking
    track = cv2.inRange(hsv, hsv_min, hsv_max)

    kernel = np.ones((2,2), np.uint8)

    img_erosion = cv2.erode(track, kernel, iterations=1)
    kernel = np.ones((6,6), np.uint8)
    img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)

    cv2.imshow('Erosion', img_erosion)
    cv2.imshow('tracking', img_dilation)

    # unedited tracking image
    # cv2.imshow('tracking', track)

    k = cv2.waitKey(1)
    if k == 27:
        break

cv2.destroyAllWindows()
