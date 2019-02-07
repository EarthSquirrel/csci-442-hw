import cv2
cap = cv2.VideoCapture(0)
cv2.namedWindow("Video")
while True:
    status, img = cap.read()
    cv2.imshow("Video", img)
    k = cv2.waitKey(1)
    if k == 27:
        break
cv2.destroyAllWindows()
