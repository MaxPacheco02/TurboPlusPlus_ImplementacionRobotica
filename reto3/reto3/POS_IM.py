#!/usr/bin/env python3

# ENCONTRAR PUNTOS EN CUADRO Y OBTENER DISTANCIA ENTRE ELLOS

import cv2
import math
import numpy as np


class POINTS:

    def __init__(self):

        # PARAMETROS DE CAMERA DE LAPTOP 720p (ESTIMADO)
        self.f = 3
        self.X = 1280
        self.A = 3.6

        # DIMENSIONES DE OBJETO (mm)
        # self.h = 90  Variantes 
        # self.w = 160
        self.Wd = 85
        self.PBUV = np.zeros((2,2),np.float32)

        # DISTANCIA A CAMARA
        self.d = 0


    def boxCal(self,mask,frame):
        cnts, hie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Wpx = 0
        x = 0 
        y = 0 
        w = 0 
        h = 0
        # Drawing rectangles 
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 2000:
                x, y, w, h = cv2.boundingRect(c)
                Wpx = math.sqrt( (w)**2 + (h)**2 )
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0),2)
        
        if(Wpx != 0):
            self.PBUV[0,:] = [x,y] 
            self.PBUV[1,:] = [x+w,y+h]

            # CALCULO DE DISTANCIA (mm)
            Wmm = (Wpx * self.A) / self.X
            self.d = (self.Wd * self.f) / Wmm

        return frame, self.d, self.PBUV


