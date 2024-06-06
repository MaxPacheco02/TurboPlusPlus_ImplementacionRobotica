#!/usr/bin/env python3

import cv2 as cv2
import numpy as np
from segmentation_functions import *

idxs = range(278)

min_threshold_slider_max = 255
img_selector_slider_max = len(idxs) - 1
brightness_slider_max = 1000
contrast_slider_max = 100
low_canny_slider_max = 255
high_canny_slider_max = 255

title_window = 'Line Detection'

min_thresh = 143
chosen_idx = 0
brightness = -277
contrast = 3.5
low_canny = 50
high_canny = 150

for chosen_idx in idxs:
    img = cv2.imread("img"+str(chosen_idx)+".png")

    w = img.shape[1] * 5
    h = img.shape[0] * 5
    img = cv2.resize(img,(w,h))

    x_c = int(w/2)
    x_t = int(w*.3)
    img = img[:: , x_c - x_t : x_c + x_t]

    img_weighted = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness)
    grayImage = cv2.cvtColor(img_weighted, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(grayImage,min_thresh,255,cv2.THRESH_BINARY_INV)

    thresh2 = pure_thresh(thresh)

    col = 0
    colsum = []

    for i in range(len(img[0])):
        for j in range(len(img)):
            col += thresh2[j][i] 
        colsum.append(int(col/j))
        col = 0

    # PORCENTAJE PARA THRESHOLD
    thr_p = 0.8

    # THRESHOLD PARA DETERMINAR LINEA
    threshold = thr_p * max(colsum)

    # print(colsum[2])

    for i in range(len(img[0])):
        for j in range(len(img)):
            if(colsum[i] < threshold):
                thresh2[j][i] = 0
            else:
                thresh2[j][i] = 255

    # MOMENTOS (PARA ENCONTRAR CENTRO DE BLOB)
    M = cv2.moments(thresh2)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # print("cX: " + str(cX) + "  cY: " + str(cY))
    else:
        cX, cY = 0, 0

    viewingFrame = img.copy()
    if cX != thresh2.shape[1]//2 - 1 or cY != thresh2.shape[0]//2 - 1:
        cv2.circle(viewingFrame,(cX, cY), 10, (0,0,255),-1)

    # frames = cv2_conc([img, img_weighted, thresh, thresh2, viewingFrame])
    frames = cv2_conc([viewingFrame])

    cv2.imshow('frames', frames)

    cv2.waitKey(100)