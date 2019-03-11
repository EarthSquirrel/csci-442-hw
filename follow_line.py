import cv2 as cv
import tkinter as tk
import maestro
import numpy as np
# import keyboardControl as kc
def quit(event):
	global win
	win.destroy()

win = tk.Tk()


win.bind('<q>', quit)
win.mainloop()
# keys = kc.KeyControl(win)



