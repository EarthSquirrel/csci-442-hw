import cv2 as cv
import numpy as np
from sys import platform as sys_pf
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")

def reduce_colors(img_orig, n_colors):
	arr = img_orig.reshape((-1,3))
	kmeans = KMeans(n_clusters=n_colors, random_state=42).fit(arr)
	labels = kmeans.labels_
	centers = kmeans.cluster_centers_
	less_colors = centers[labels].reshape(img_orig.shape).astype('uint8')
	cv.imshow('Color Reduction', less_colors)
	cv.imwrite('10_color_reduc.jpg', less_colors)


img = cv.imread('./10_color_reduc.jpg', cv.IMREAD_COLOR)
blur = cv.blur(img,(3,3))
edges = cv.Canny(blur,10,300)
ret, thresh = cv.threshold(edges,0,255,0)
img2, contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
cv.drawContours(edges, contours, -1, (0,255,0), 3)
cv.imshow('Edges', edges)





cv.waitKey(0)