import cv2 as cv
import tkinter as tk
import maestro
import numpy as np
# import keyboardControl as kc
def quit(event):
	global win
	win.destroy()


def arrow(key):
	print(key.keycode)
	print('\n')

win = tk.Tk()
win.bind('<Up>', arrow)
win.bind('<Left>', arrow)
win.bind('<Down>', arrow)
win.bind('<Right>', arrow)
win.bind('<space>', arrow)
win.bind('<q>', quit)
win.mainloop()
# keys = kc.KeyControl(win)





