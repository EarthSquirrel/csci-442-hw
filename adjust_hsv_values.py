from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2 as cv
import sys
import maestro
import numpy as np

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 10
rawCapture = PiRGBArray(camera, size=(640,480))
servo = maestro.Controller()

HEADTILT = 4


max_move = 5200  # 5400
max_turn = 7200  # left
min_turn = 4800  # right
max_head_turn = 7500 # max 7900
min_head_turn = 4000 # min 1510
max_time = 5

motors = 6000
turn = 6000
body = 6000
headTurn = 6000
headTilt = 6000


pink_min = np.array([164, 65, 252])
pink_max = np.array([166, 75, 255])

yellow_min = np.array([0, 40, 255])
yellow_max = np.array([30, 80, 255])

white_min = np.array([100, 5, 230])
white_max = np.array([130, 20, 255])

#yellow_ice_min = np.array([30, 135, 200])
#yellow_ice_max = np.array([35, 170, 220])
yellow_ice_min = np.array([30, 115, 200])
yellow_ice_min = np.array([60, 220, 240])

green_ice_min = np.array([20, 150, 120])
green_ice_max = np.array([100, 255, 220])
#green_ice_min = np.array([20, 160, 120])
#green_ice_max = np.array([75, 225, 190])


cv.namedWindow('hsv', cv.WINDOW_NORMAL)

def get_hsv_val(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        vals = image[y][x]
        # set track bar to -10 + 10 of click
        adjust = 25
        cv.setTrackbarPos('H Min', 'hsv', vals[0]-adjust)
        cv.setTrackbarPos('H Max', 'hsv', vals[0]+adjust)
        cv.setTrackbarPos('S Min', 'hsv', vals[1]-adjust)
        cv.setTrackbarPos('S Max', 'hsv', vals[1]+adjust)
        cv.setTrackbarPos('V Min', 'hsv', vals[2]-adjust)
        cv.setTrackbarPos('V Max', 'hsv', vals[2]+adjust)

def nothing(x):
    pass

cv.createTrackbar('H Min','hsv',0,255, nothing)
cv.createTrackbar('H Max','hsv',0,255, nothing)
cv.createTrackbar('S Min','hsv',0,255, nothing)
cv.createTrackbar('S Max','hsv',0,255, nothing)
cv.createTrackbar('V Min','hsv',0,255, nothing)
cv.createTrackbar('V Max','hsv',0,255, nothing)

cv.setMouseCallback("hsv", get_hsv_val)




def prepareImage(img):
    kernel = np.ones((10,10), np.uint8)
    blur = cv.blur(img,(7,7))
    erosion = cv.erode(blur, kernel, iterations=4)
    dilation = cv.dilate(erosion, kernel, iterations=2)
    return dilation


def contourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h


def find_contours(color, img):
    prep_img = prepareImage(img)
    hsv_img = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    hsv_img = prepareImage(hsv_img)
    thresh = cv.inRange(hsv_img, green_ice_min, green_ice_max)
    thresh = prepareImage(thresh)
    edges = cv.Canny(thresh, 35, 150, L2gradient=True)
    edges = cv.blur(edges, (7,7))

    if color == "green":
        min_hsv = green_ice_min
        max_hsv = green_ice_max
    elif color == "yellow":
        min_hsv = yellow_min
        max_hsv = yellow_max
    elif color == "pink":
        min_hsv = pink_min
        max_hsv = pink_max


    filtered_img = cv.inRange(hsv_img, min_hsv, max_hsv)
    edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)

    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    contoursS = sorted(contours, key=lambda x: contourArea(x))

    return contoursS, len(contoursS)





try:
# capture frames from the camera
    servo.setTarget(HEADTILT, 1510)
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        hsv_img = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        filtered_img = cv.inRange(hsv_img, green_ice_min, green_ice_max)
        edges = cv.Canny(filtered_img, 35, 150, L2gradient=True)
        edges = cv.blur(edges, (7,7))
        contours, cnt_count = find_contours("green", image)
        cv.drawContours(image, contours, -1, (0,0,255), 3)
        print("Contour count:", cnt_count)
        cv.imshow("Image", image)
        cv.imshow("hsv", hsv_img)
        cv.imshow("Edges", edges)
        cv.imshow("Filtered", filtered_img)




        key = cv.waitKey(1) & 0xFF
        if key == ord("c"):
            cv.imwrite('yellow_hsv.png', image)


        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)

except: # catch *all* exceptions
    e = sys.exc_info()
    print(e)
    # keys.arrow (65)
    END_PROGRAM = True
    #stop()
    servo.setTarget(HEADTILT, 6000)
    print('Stopped motors due to an error')



