import cv2
import numpy as np

cap = cv2.VideoCapture(0)
cv2.namedWindow("Video")

def get_hsv_val(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(int(cap[x,y]))

        
cv2.setMouseCallback("Video", get_hsv_val)

while True:
    status, img = cap.read()
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Video", 600,400)
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
    
    cv2.resizeWindow('img',600,400)
    cv2.resizeWindow('mask',600,400)
    cv2.resizeWindow('res',600,400)
    
    cv2.imshow('img',img)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    k = cv2.waitKey(1)
    if k == 27:
        break
cv2.destroyAllWindows()
