import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt
# import colormap
# from colormap import rgb2hex
cv.namedWindow("Color circles")

color_labels = {int('FF0000', 16): 'blue', int('00FF00', 16): 'green',
                int('0000FF', 16): 'red', int('00FFFF', 16): 'yellow',
                int('2a2aa5', 16): 'brown', int('99A5FF', 16): 'orange'}
color_rgb = {'blue': [255, 0, 0], 'green': [0, 255, 0], 'red': [0, 0, 255],
             'yellow': [0, 255, 255], 'brown': [42, 42, 165], 'orange':
             [0, 165, 255]}
def get_hsv_val(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        vals = img[y][x]
        print("colors are: ", vals)

cv.setMouseCallback("Color circles", get_hsv_val)

def get_label(rgb):
    min_dist = np.Infinity
    label = ''
    color_values = list(color_rgb.keys())

    for color in color_values:
        dist = np.sqrt(sum([(rgb[i]-color_rgb[color][i])**2 for i in range(3)]))
        if dist < min_dist:
            min_dist = dist
            label = color

    return label

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
###cv.imshow('blur', blur)

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
    colors.append(mc)
    # print(mc)
    label = get_label(mc)
    print(label)
    mc2 = mc.copy()
    mc = color_rgb[label]
    print(mc2)
    print(mc)
    tmp = cv.cvtColor(tmp, cv.COLOR_GRAY2RGB)
    cv.circle(tmp, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)
    ### cv.imshow('tmp {}'.format(label), tmp)
    cv.circle(color_img, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)
    cv.putText(color_img, str(mc2), (int(x),int(y)), cv.FONT_HERSHEY_SIMPLEX,.3,
              (0,255,0),lineType=cv.LINE_AA)

colors.sort()
# print(colors)
#for c in colors:
#    print(rgb2hex(c))

### cv.imshow('White Cirlces', white_img)
cv.imshow('Color circles', color_img)
lab_img = cv.cvtColor(color_img, cv.COLOR_RGB2Lab)
# cv.imshow('labels', lab_img)

x_text = int(width) - 50
y_text = int(height) - 10
output = "Number M&M: {}".format(len(keypoints))
# cv.putText(img,"Hello World!!!", (int(width-10), int(height-10)), cv.FONT_HERSHEY_SIMPLEX,3,
cv.putText(img,output, (10, y_text), cv.FONT_HERSHEY_SIMPLEX,1,
           (255,255,255),2,cv.LINE_AA)

cv.imshow('image', img)

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
