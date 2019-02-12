import cv2 as cv
cap = cv.VideoCapture(0)
cv.namedWindow("Video")
cv.namedWindow("HSV")
while True:
    status, img = cap.read()
    cv.imshow("Video", img)
    k = cv.waitKey(1)
    if k == 27:
        break
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    cv.imshow("HSV", hsv)
cv.destroyAllWindows()
