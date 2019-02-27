import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
# import colormap
# from colormap import rgb2hex

color_labels = {int('FF0000', 16): 'red', int('00FF00', 16): 'green',
                int('0000FF', 16): 'blue', int('FFFF00', 16): 'yellow',
                int('800000', 16): 'brown'}
color_rgb = {'red': [255, 0, 0], 'green': [0, 255, 0], 'blue': [0, 0, 255],
             'yellow': [255, 255, 0], 'brown': [128, 0, 0]}


def get_label(rgb):
    # convert rgb to hex value
    hex_str = "{:02x}{:02x}{:02x}".format(rgb[0], rgb[1], rgb[2])
    diff = []
    # turn into hex and find closest number
    num = int(hex_str, 16)
    color_values = (list(color_labels.keys()))
    for i in color_values:
        diff.append(abs(num-i))
    # print('index: ', min(diff), 'min: ', diff.index(min(diff)))
    val = diff.index(min(diff))
    close = color_values[val]
    return val, color_labels[close]

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

def rgb2hex(rgb):
    return "{:02x}{:02x}{:02x}".format(rgb[0], rgb[1], rgb[2])

def hex2rgb(hexcode):
    return tuple(map(ord,hexcode[1:].decode('hex')))

# Draw new smaller circles
width, height, channel = img.shape
white_img = 0* np.ones((width, height, 1), np.uint8)
color_img = 0* np.ones((width, height, 1), np.uint8)
color_img = cv.cvtColor(color_img, cv.COLOR_GRAY2RGB)

colors = []
for k in keypoints:
    tmp = 0* np.ones((width, height, 1), np.uint8)
    (x, y) = k.pt
    radius = int(k.size-5)
    cv.circle(white_img, (int(x), int(y)), 0, (255, 255, 255), radius)
    # Make a temporary image of just that color
    cv.circle(tmp, (int(x), int(y)), 0, (255, 255, 255), radius)
    mc = cv.mean(img,mask = tmp)
    mc = [int(m) for m in mc]
    # print(mc)
    val,label = get_label(mc)
    print(label)
    print(mc)
    mc = color_rgb[label]
    print(mc)
    tmp = cv.cvtColor(tmp, cv.COLOR_GRAY2RGB)
    cv.circle(tmp, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)
    cv.imshow('tmp {}'.format(label), tmp)
    cv.circle(color_img, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)


cv.imshow('White Cirlces', white_img)
cv.imshow('Color circles', color_img)
lab_img = cv.cvtColor(color_img, cv.COLOR_RGB2Lab)
# cv.imshow('labels', lab_img)




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
