
import numpy as np
import cv2 as cv

face_cascade = cv.CascadeClassifier('data/haarcascades/haarcascade_frontalface_default.xml')
eye_cascade = cv.CascadeClassifier('data/haarcascades/haarcascade_eye.xml')

cv.namedWindow("Image", cv.WINDOW_NORMAL)

img = cv.imread("people.jpg", cv.IMREAD_COLOR)

gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
faces = face_cascade.detectMultiScale(gray, 1.3, 5)
print(faces)
for (x,y,w,h) in faces:
    cv.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
    roi_gray = gray[y:y+h, x:x+w]
    roi_color = img[y:y+h, x:x+w]
    eyes = eye_cascade.detectMultiScale(roi_gray)
    for (ex,ey,ew,eh) in eyes:
        cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
##eyes = eye_cascade.detectMultiScale(gray)
##for (ex,ey,ew,eh) in eyes:
##    cv.rectangle(img,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)

cv.imshow('Image',img)
cv.waitKey(0)
cv.destroyAllWindows()

