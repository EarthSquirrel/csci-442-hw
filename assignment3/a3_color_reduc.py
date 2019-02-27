import cv2 as cv
import numpy as np
from sys import platform as sys_pf
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")

def reduce_colors(img_orig, n_colors, filename):
	arr = img_orig.reshape((-1,3))
	kmeans = KMeans(n_clusters=n_colors, random_state=42).fit(arr)
	labels = kmeans.labels_
	centers = kmeans.cluster_centers_
	less_colors = centers[labels].reshape(img_orig.shape).astype('uint8')
	cv.imshow('Color Reduction', less_colors)
	cv.imwrite(filename + '.jpg', less_colors)
	return less_colors


img = cv.imread('./imagesWOvideo/three.jpg', cv.IMREAD_COLOR)
color_reduc = reduce_colors(img, 5, 'five_color_reduc_im3')
cv.imshow("Color reduction", color_reduc)





cv.waitKey(0)