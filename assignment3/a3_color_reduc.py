import cv2 as cv
import numpy as np
from sys import platform as sys_pf
from matplotlib import pyplot as plt
from sklearn.cluster import KMeans
if sys_pf == 'darwin':
    import matplotlib
    matplotlib.use("TkAgg")

n_colors = 10

img_orig = cv.imread('./imagesWOvideo/one.jpg', cv.IMREAD_COLOR)

arr = img_orig.reshape((-1,3))
kmeans = KMeans(n_clusters=n_colors, random_state=42).fit(arr)
labels = kmeans.labels_
centers = kmeans.cluster_centers_
less_colors = centers[labels].reshape(img_orig.shape).astype('uint8')



cv.imshow('Color Reduction', less_colors)
cv.imwrite('10_color_reduc.jpg', less_colors)


cv.waitKey(0)