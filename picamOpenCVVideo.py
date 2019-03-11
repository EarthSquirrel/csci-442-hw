# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
import numpy as np
import sys
import tkinter as tk
import keyboardControl


# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 5 # 15  #   32
rawCapture = PiRGBArray(camera, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

# import and make the key controls here... Please!
win = tk.Tk()
keys = keyboardControl.KeyControl(win)

win.bind('<Up>', keys.arrow)
win.bind('<Left>', keys.arrow)
win.bind('<Down>', keys.arrow)
win.bind('<Right>', keys.arrow)
win.bind('<space>', keys.arrow)
win.bind('<z>', keys.waist)
win.bind('<c>', keys.waist)
win.bind('<w>', keys.head)
win.bind('<s>', keys.head)
win.bind('<a>', keys.head)
win.bind('<d>', keys.head)
win.mainloop()

def myContourArea(cnt):
   x,y,w,h = cv.boundingRect(cnt)
   return w*h

# control the motors
def go_straigh():
    pass

def turn_right():
    pass

def turn_left():
    pass

def stop():
    pass

started = False
prev_cog = (0, 0)
cog = (0, 0)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    if started:
        prev_cog = cog
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    width, height, channel = img.shape
    img = img[int(1*height/3):height, 20:width-20]
    # __,new_image = cv.threshold(img, 0, 400, cv.THRESH_BINARY)
    # cv.imshow("newimg", new_image)
    blur = cv.blur(img,(7,7))

    kernel = np.ones((10,10), np.uint8)

    img_erosion = cv.erode(blur, kernel, iterations=2)
    img_dilation = cv.dilate(img_erosion, kernel, iterations=3)
    # color filtering stuff, save for later
    hsv = cv.cvtColor(img_dilation, cv.COLOR_BGR2HSV)
    # hsv = img_dilation.copy()

    hsv_min, hsv_max = (0, 40, 90), (75, 250, 255)
    color_filter = cv.inRange(hsv, hsv_min, hsv_max)
    # pic = cv.Canny(hsv, 150, 170)

    # cv.imshow('hsv',color_filter) # hsv)

    edges = cv.Canny(color_filter, 35, 150, L2gradient=True)
    cv.imshow('edges', edges)

    contours, hierarchy = cv.findContours(edges, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    width, height, channel = img.shape
    tours = 255 * np.ones((width,height,1), np.uint8)
    thresh = img.copy()

    cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
    # print(len(contours))
    # contours = cntsSorted[-3]

    contoursS = sorted(contours, key=lambda x: myContourArea(x))
    contoursS.reverse()
    # if myContourArea(contoursS[-1]) == 0:
    #    del contoursS[-1]

    for cnt in contours:
        x,y,w,h = cv.boundingRect(cnt)
        cv.rectangle(thresh, (x,y), (x+w,y+h,), (0,255,0), 2)
        break

    M = cv.moments(cnt)
    # print(myContourArea(cnt))
    if M['m00'] == 0:
        div_by =0.1
    else:
        div_by = M['m00']
    cx = int(M['m10']/div_by)
    cy = int(M['m01']/div_by)
    # print('({}, {})'.format(cx, cy))

    cog = (cx, cy)
    print(cog[3])
    if started:
        angle = np.math.atan2(np.linalg.det([cog,prev_cog]),np.dot(cog,prev_cog))
        # print(angle)  # np.degrees(angle))
        """
        if angle == 0:
            # print('straight')

        elif angle < 0:
            print('left')
        else:
            print('right')
        """
    cv.rectangle(thresh,  (cx,cy), (cx+15, cy+15),(0,0,255), 2)


    cv.drawContours(tours, contours, -1, (0,0,255), -1)
    cv.imshow('Contours', thresh)

    started = True


    # show the frame
    # cv.imshow("Frame", pic)
    key = cv.waitKey(1) & 0xFF

    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)

    # if the `q` key was pressed, break from the loop
    if key == ord("q"):
        break
"""
except: # catch *all* exceptions
    e = sys.exc_info()[0]
    keys.arrow (65)
    print('Stopped motors due to an error')
"""
