#!/usr/bin/env python3

import cv2
import glob
import numpy as np
import json
import os

from ament_index_python.packages import get_package_share_directory

# TOMAR FOTOS Y CALIBRAR CAMARA

class CAM:

    def __init__(self):
        # MATRICES Y VECTORES OBTENIDOS DE CALIBRACION DE CAMARA
        self.CamMat = 0
        self.CamTra = 0
        self.CamRot = 0
        self.CamDis = 0

        # HSV PARA MASCARA
        self.h_min = 0
        self.h_max = 0
        self.s_min = 0
        self.s_max = 0
        self.v_min = 0
        self.v_max = 0

        self.mask = np.zeros([720,1280])


    # TOMAR FOTOS DE CHESSBOARD PARA CALIBRACION
    def imageCapture(self):
        IMG = cv2.VideoCapture(0)
        num = 0
        while IMG.isOpened():
            ret, frame = IMG.read()
            k = cv2.waitKey(1)
            if k == ord('p'): 
                cv2.imwrite('./chessboard/chess' + str(num) + '.png', frame)
                print("images saved!")
                num += 1
            elif k == ord('q'):
                break
            cv2.imshow('Image',frame)
        IMG.release()
        cv2.destroyAllWindows()


    def cameraCalibrate(self):
        # NUMERO DE ESQUINAS EN CHESSBOARD DE CALIBRACION
        chessSize = (5,5)
        # TAMAÑO DE CADA CUADRO EN MM
        chessSq = 31
        # TERMINATION CRITERIA
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # PUNTOS PARA PLANOS DE CALIBRACION
        objp = np.zeros((chessSize[0]*chessSize[0],3), np.float32)
        objp[:,:2] = np.mgrid[0:chessSize[0],0:chessSize[1]].T.reshape(-1,2)
        # ASIGNAR MEDIDAS REALES A OBJP (MM)
        objp = objp * chessSq
        # ARREGLOS DE PUNTOS EN IMAGEN Y CORRESPONDENCIAS EN EL MUNDO
        objpoints = [] # 3d 
        imgpoints = [] # 2d

        # OBTENER PATH DE DIRECTORIO DE FOTOS (EN DIRECTORIO SHARE DEL PAQUETE)
        share_dir = get_package_share_directory('reto3')
        chessboard_path = os.path.join(share_dir, 'resources')

        # IMPORTAR IMAGENES PARA CALIBRACION
        images = glob.glob(os.path.join(chessboard_path, '*.png'))

        for fname in images:
            img = cv2.imread(fname)

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, chessSize, None)

            # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                
                corners2 = cv2.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                
                # Draw and display the corners
                cv2.drawChessboardCorners(img, chessSize, corners2, ret)
                cv2.imshow('Imagen', img)
                cv2.waitKey(0)

        cv2.destroyAllWindows()

        #CALIBRACION DE CAMARA
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, None)
        self.CamMat = mtx
        self.CamDis = dist
        self.CamRot = rvecs
        self.CamTra = tvecs
    

    # Returns empty for trackbar
    def empty(self,IMG):
        pass

    def colorCal(self):
        # Open camera
        cap = cv2.VideoCapture(0)

        # OBTENER PATH DE DIRECTORIO DE JSON (EN DIRECTORIO SHARE DEL PAQUETE)
        share_dir = get_package_share_directory('reto3')
        json_path = os.path.join(share_dir, 'config','colors.json')

        # ABRIR JSON FILE
        f = open(json_path,'r') # Open json file
        data = json.load(f) # Load data from json
        f.close() # close file
        color = data["type"] # chosing color

        # Trackbars for calibration 
        cv2.namedWindow("Trackbar")
        cv2.resizeWindow("Trackbar",600,300)
        cv2.createTrackbar("hue_min","Trackbar",data[color]["hue_low"],180,self.empty)   
        cv2.createTrackbar("hue_max","Trackbar",data[color]["hue_up"],180,self.empty)
        cv2.createTrackbar("sat_min","Trackbar",data[color]["sat_low"],255,self.empty)
        cv2.createTrackbar("sat_max","Trackbar",data[color]["sat_up"],255,self.empty)
        cv2.createTrackbar("val_min","Trackbar",data[color]["val_low"],255,self.empty)
        cv2.createTrackbar("val_max","Trackbar",data[color]["val_up"],255,self.empty)
        cv2.createTrackbar("low_threshold","Trackbar",0,255,self.empty)
        cv2.createTrackbar("up_threshold","Trackbar",100,255,self.empty)

        while True:
            # Read frame from camera
            ret, frame = cap.read()
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Values from trackbar
            self.h_min = cv2.getTrackbarPos("hue_min", "Trackbar")
            self.h_max = cv2.getTrackbarPos("hue_max", "Trackbar")
            self.s_min = cv2.getTrackbarPos("sat_min", "Trackbar")
            self.s_max = cv2.getTrackbarPos("sat_max", "Trackbar")
            self.v_min = cv2.getTrackbarPos("val_min", "Trackbar")
            self.v_max = cv2.getTrackbarPos("val_max", "Trackbar")

            mask = self.procMask(hsv)

            cv2.imshow("Mask", mask)
            k = cv2.waitKey(1)
            if(k==ord('q')):
                break
        cap.release()
        cv2.destroyAllWindows()


    def procMask(self,hsv):
        # Range values of HSV
        lower = np.array([self.h_min, self.s_min, self.v_min])
        upper = np.array([self.h_max, self.s_max, self.v_max])
        # Mask with HSV values 
        self.mask = cv2.inRange(hsv, lower, upper)
        # Cleaning mask
        kernel = np.ones((5, 5), np.uint8) 
        self.mask = cv2.erode(self.mask, kernel, iterations=1)
        self.mask = cv2.dilate(self.mask, kernel, iterations=1)
        self.mask = cv2.dilate(self.mask, kernel, iterations=1)
        self.mask = cv2.erode(self.mask, kernel, iterations=1)
        return self.mask