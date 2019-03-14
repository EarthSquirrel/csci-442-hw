from picamera import PiCamera
from time import sleep
import cv2 as cv

camera = PiCamera()
camera.start_preview()
sleep(10)
camera.stop_preview()
