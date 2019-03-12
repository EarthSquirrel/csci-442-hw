import cv2 as cv
import numpy as np
from sys import platform as sys_pf
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
from sklearn.cluster import MiniBatchKMeans
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


def _reduce_colors(img_orig, n_clusters):
	(h,w) = img_orig.shape[:2]
	# image = cv.cvtColor(img_orig, cv.COLOR_BGR2LAB)
	image = img_orig.reshape((img_orig.shape[0] * img_orig.shape[1],3))
	clt = MiniBatchKMeans(n_clusters)
	labels = clt.fit_predict(image)
	quant = clt.cluster_centers_.astype("uint8")[labels]
	quant = quant.reshape((h, w, 3))
	image = image.reshape((h, w, 3))
	# quant = cv.cvtColor(quant, cv.COLOR_LAB2BGR)
	# image = cv.cvtColor(image, cv.COLOR_LAB2BGR)
	cv.imshow("image", image)
	cv.imshow("quant", quant)


img = cv.imread('./imagesWOvideo/three.jpg', cv.IMREAD_COLOR)
# color_reduc = reduce_colors(img, 5, 'five_color_reduc_im3')
# cv.imshow("Color reduction", color_reduc)

_reduce_colors(img, 4)



cv.waitKey(0)