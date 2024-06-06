#!/usr/bin/env python3
import cv2
import numpy as np

idxs = [0,2,3,4,5,6,7,9]
for chosen_idx in idxs:
    IMG_N = cv2.imread("img"+str(idxs[chosen_idx])+".png")

    Dim = IMG_N.shape[:2]

    # CENTRO DE IMAGEN EN X
    x_c = int(Dim[1]*0.5)

    # PORCENTAGE A LOS LADOS X
    x_t = int(Dim[1]*1. / 2)


    # RECORTAR IMAGEN
    frame = IMG_N[:: , x_c - x_t : x_c + x_t]

    # DIMENSIONES DE IMAGEN RECORTADA
    Dim_f = frame.shape[:2]

    # CENTRO DE IMAGEN NUEVA X
    xn_c = int(Dim_f[1]*0.5)

    brightness = 10
    contrast = 1
    frame = cv2.addWeighted(frame, contrast, np.zeros(frame.shape, frame.dtype), 0, brightness)
    grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    ret,thresh = cv2.threshold(grayImage,140,255,cv2.THRESH_BINARY_INV)

    col = 0
    colsum = []

    for i in range(0, Dim_f[1]):
        for j in range(0, Dim_f[0]):
            col += thresh[j][i] 
        colsum.append(int(col/j))
        col = 0

    # PORCENTAJE PARA THRESHOLD
    thr_p = 0.8

    # THRESHOLD PARA DETERMINAR LINEA
    threshold = thr_p * max(colsum)

    # print(colsum[2])

    for i in range(0, Dim_f[1]):
        for j in range(0, Dim_f[0]):
            if(colsum[i] < threshold):
                thresh[j][i] = 0
            else:
                thresh[j][i] = 255

    # MOMENTOS (PARA ENCONTRAR CENTRO DE BLOB)
    M = cv2.moments(thresh)

    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        # print("cX: " + str(cX) + "  cY: " + str(cY))
    else:
        cX, cY = 0, 0

    cv2.circle(frame,(cX, cY), 10, (0,0,255),-1)
    cv2.imshow('gray frame', grayImage)
    cv2.imshow('original frame', frame)
    cv2.imshow('processed frame', thresh)
    key = cv2.waitKey(10000)
