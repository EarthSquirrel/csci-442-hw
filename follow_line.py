import cv2 as cv
import tkinter as tk
import maestro
import keyboardControl

keys = keyboardControl.KeyControl()
servo = maestro.Controller()
#servo.setAccel(0,4)
#servo.setTarget(0, 0)# ,6000)
#servo.setSpeed(0,5)
#x = servo.getPosition(1)
#print(x)

servo.close()

