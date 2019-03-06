import cv2 as cv
import numpy as np
import sys
cv.namedWindow('Counting M&Ms', cv.WINDOW_NORMAL)

color_rgb = {'blue': [255, 0, 0], 'green': [0, 255, 0], 'red': [0, 0, 230],
             'yellow': [0, 255, 255], 'brown': [0,0,53], 'orange':
             [0, 165, 255]}

# old brown 42,42,165
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

print(sys.argv)
if len(sys.argv) <= 1:
    img = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)
    # img = cv.imread('./imagesWOvideo/two.jpg', cv.IMREAD_COLOR)
    # img = cv.imread('./imagesWOvideo/three.jpg', cv.IMREAD_COLOR)
    img = cv.imread('./imagesWOvideo/four.jpg', cv.IMREAD_COLOR)
else:
    img = cv.imread(sys.argv[1], cv.IMREAD_COLOR)

final_img = img.copy()

blur = cv.blur(img,(3,3))

kernel = np.ones((5,5), np.uint8)

img_erosion = cv.erode(blur, kernel, iterations=2)
img_dilation = cv.dilate(img_erosion, kernel, iterations=2)

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
circles = cv.HoughCircles(gray,cv.HOUGH_GRADIENT,1, 20,
                           param1=150,param2=35,minRadius=15,maxRadius=75)

circles = np.uint16(np.around(circles))
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(img,(i[0],i[1]),2,(0,0,255),3)
cv.imshow('Counting M&Ms',img)
"""

# Set up parameters for blob detection

params = cv.SimpleBlobDetector_Params()


# Change thresholds
params.minThreshold = 2# 2
params.maxThreshold = 500
params.filterByCircularity = True
params.minCircularity = 0.3
params.maxCircularity = 1.5


detector = cv.SimpleBlobDetector_create(params)

keypoints = detector.detect(edges)
keypoints_im = cv.drawKeypoints(edges, keypoints, np.array([]), (0, 255, 0),
                                cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

radius_avg = 0
for k in keypoints:
    radius_avg += int(k.size)
print('Total keypoints: {} Radius avg {}, compare to {}'.format(len(keypoints),
       radius_avg, radius_avg-10))

radius_avg = radius_avg/len(keypoints)

color_counts = {'red': 0, 'green': 0, 'blue': 0, 'yellow': 0, 'brown':0,
                'orange': 0}

# Draw new smaller circles
width, height, channel = img.shape
white_img = 0* np.ones((width, height, 1), np.uint8)
color_img = 0* np.ones((width, height, 1), np.uint8)
color_img = cv.cvtColor(color_img, cv.COLOR_GRAY2RGB)
checker = img.copy()
real_mms = []
removed_rad = 'Removing keypoints: '

for k in keypoints:
    tmp = 0* np.ones((width, height, 1), np.uint8)
    (x, y) = k.pt
    radius = int(k.size-5)
    if radius > radius_avg/2:
        real_mms.append(k)
        cv.circle(white_img, (int(x), int(y)), 0, (255, 255, 255), radius)
        # Make a temporary image of just that color
        cv.circle(tmp, (int(x), int(y)), 0, (255, 255, 255), radius)
        mc = cv.mean(img,mask = tmp)
        mc = [int(m) for m in mc]
        label = get_label(mc)
        mc2 = mc.copy()
        mc = color_rgb[label]

        # increment color count
        color_counts[label] = color_counts[label] + 1
        cv.circle(final_img, (int(x), int(y)),int(radius/2), (102,0,255), 3)

        cv.circle(color_img, (int(x), int(y)), 0, (mc[0], mc[1], mc[2]), radius)
    else:
        removed_rad += '({}, {}): r={}, '.format(int(x), int(y), radius)

print(removed_rad)


x_text = int(width) - 50
y_text = 10  # int(2*height/3)

output = 'Total: {}'.format(len(real_mms))
cv.putText(final_img,output, (10, y_text), cv.FONT_HERSHEY_SIMPLEX,.8,
           (0,0,0),1,cv.LINE_AA)
print(output)

# print colors to command line for now
for key in list(color_counts.keys()):
    y_text += 20
    output = '{}: {}'.format(key, color_counts[key])
    print(output)
    cv.putText(final_img,output, (10, y_text), cv.FONT_HERSHEY_SIMPLEX,.65,
               (255,255,255),2,cv.LINE_AA)


# cv.imshow('image', img)
###cv.imshow('blur', blur)
# cv.imshow('Erosion', img_erosion)
# cv.imshow('tracking', img_dilation)
# cv.imshow('White Cirlces', white_img)
# cv.imshow('Color circles', color_img)
cv.imshow('Counting M&Ms', final_img)

"""
cv.waitKey(0)
