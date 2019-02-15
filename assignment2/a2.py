import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.namedWindow("Video")


def nothing(x):
    pass

def get_hsv_val(event, x, y, flags, param):
    __, image = cap.read()
    if event == cv2.EVENT_LBUTTONDOWN:
        print(image[y][x])

        
cv2.setMouseCallback("Video", get_hsv_val)

cv2.createTrackbar('Blue Min', 'Video', 0,255,nothing)
cv2.createTrackbar('Blue Max', 'Video', 0,255,nothing)


while True:
    status, img = cap.read()
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Video", 600,600)
    cv2.imshow("Video", img)

    # Convert to hsv
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    res = cv2.bitwise_and(img,img, mask=mask)
    
    cv2.namedWindow('img', cv2.WINDOW_NORMAL)
    cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
    cv2.namedWindow('res', cv2.WINDOW_NORMAL)
    
    cv2.resizeWindow('img',600,600)
    cv2.resizeWindow('mask',600,600)
    cv2.resizeWindow('res',600,600)
    
    cv2.imshow('img',img)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    k = cv2.waitKey(1)
    if k == 27:
        break
cv2.destroyAllWindows()
