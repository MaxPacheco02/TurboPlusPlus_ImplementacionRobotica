# Posición de punt en plano de imagen a coordenadas de mundo

import cv2
import glob
import numpy as np
# from math import ceil
from Draw_ax import Draw

def printParams(ret, mtx, dist, rvecs, tvecs, dim):
    print("\nmtx ",dim," : \n",mtx)
    print("\ndist ",dim," : \n", dist)
    print("\nrvecs ",dim," : \n", rvecs)
    print("\ntvecs ",dim," : \n", tvecs)
    print("\n-----------------------------------")


def worldToPlane(W,H,K):
    # CONSTRUIR MATRIZ INTRINSECA
    M = np.concatenate((K,np.array([[0,0,0]]).T), axis=1)
    # DE REFERENCIAL DE MUNDO A REFERENCIAL DE CAMARA
    C = H @ W
    # DE REFERENCIAL DE CAMARA A PLANO DE IMAGEN
    V = M @ C
    # PUNTO EN PIXELES EN PLANO DE IMAGEN (u,v) -> (X,Y)
    u = V[0]/V[2]
    v = V[1]/V[2]
    return [round(np.ravel(u)[0],0),round(np.ravel(v)[0],0)]


# CALLBACK DE CLICK DE MOUSE
def select_point(event, x, y, flags, param):
    # EVENTO DE CLICK IZQUIERDO DE MOUSE
    if event == cv2.EVENT_LBUTTONDOWN:
        global PUV 
        # OBTENER COORDENADAS DE PIXEL SELECCIONADO
        arr = np.array([[x, y]], np.float32)
        print("\n(",x," , ",y,")")
        PUV[0][0] = arr[0][0]
        # print("\nP_apos[0][0]: ",P_apos[0][0])
        # print("\narr[0][0]: ",arr[0][0])
        PUV[1][0] = arr[0][1]
        PUV[2][0] = 1
        # print("\nP_apos[1][0]: ",P_apos[1][0])
        # print("\nPUV: \n",PUV)
        cv2.circle(param, (x, y), 5, (0, 255, 0), -1) 
        cv2.imshow("Imagen", param)


def track():
    IMG = cv2.imread("./chessboard/Pic.png")
    cv2.namedWindow("Imagen")
    while True:
        cv2.imshow("Imagen", IMG)
        cv2.setMouseCallback("Imagen", select_point, IMG)
        k = cv2.waitKey(1)
        if(k==ord('q')):
            break
    cv2.destroyAllWindows()


# INICIALIZAR OBJETO DE PLOT
plot = Draw()

# NUMERO DE ESQUINAS EN CHESSBOARD DE CALIBRACION
chessSize = (5,5)

# TAMAÑO DE CADA CUADRO EN MM
chessSq = 31

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessSize[0]*chessSize[0],3), np.float32)
objp[:,:2] = np.mgrid[0:chessSize[0],0:chessSize[1]].T.reshape(-1,2)

# ASIGNAR MEDIDAS REALES A OBJP (MM)
objp = objp * chessSq

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('./chessboard/*.png')
# images = ['Pic.png']

cv2.namedWindow("Imagen")
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


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, None)
T = tvecs[4]
# 'X' EN REFERENCIAL DE MUNDO DE IMAGEN[4] ES NEGATIVA
T[0][0] = T[0][0] * -1
R, _ = cv2.Rodrigues(rvecs[4])

# R = np.array([[100,200,300],[300,200,100],[50,67,100]],np.float32)

print("\nR = \n",R)
print("\nrvecs[4] = \n",rvecs[4])
print("\ntvecs[4] = \n",tvecs[4])

#TRACK (EL PAPER NO LO DEJA MUY CLARO QUE DIGAMOS CARAJO)
# -----------------------------------------------------------------------------------
PUV = np.zeros((3,1))

# CONSTRUIR MATRIZ DE VALORES EXTRINSECOS P 4x4 (MATRIZ DE TRANSFORMACION HOMOGENEA)
H = np.concatenate((np.concatenate((R , T), axis=1),[[0,0,0,1]]), axis=0)

print("\nH = \n",H)

# MATRIZ DE CALIBRACION K
K = mtx #np.concatenate((mtx, [[0],[0],[0]]), axis=1) 

print("\nmtx = \n",mtx)

# MATRIZ DE CAMARA COMPLETA CON PARAM INTRINSECOS Y EXTRINSECOS
#M = K @ P

# ---------------------
#   P' = M * Pw        ------>  P' = (K * [R T]) * Pw
#   P' * inv(M) = Pw
# ---------------------


# OBTENER PUNTOS EN PLANO DE IMAGEN
track()

# DE PLANO DE IMAGEN A MUNDO 
# -----------------------------------------------------------------------------------

# DISTANCIA DE PUNTO A LA REFERENCIAL DE CAMARA (Por ahora estimada)
Zc = -300

C_norm = np.linalg.inv(K) @ PUV 
C = C_norm * Zc 
# AGREGRAR 1 A CORDS DE REFERENCIAL DE CAMARA [Xc;Yc;Zc;1]
C = np.concatenate((C,[[1]]), axis = 0)
print("\nR.T = \n",R.T)
print("\n-(R.T@T) = \n",-(R.T@T))
# CONSTRUIR INVERSA DE LA MATRIZ HOMOGENEA
H_inv = np.concatenate((np.concatenate((R.T,-(R.T@T)), axis=1), [[0,0,0,1]]), axis=0)
print("\nH_inv = \n",H_inv)
# PUNTO EN EL MUNDO
PW = H_inv @ C

print("\nPW = \n",PW)

# -----------------------------------------------------------------------------------



# PUNTO EN EL ESPACIO 3D
W = np.array([[31,0,0,1]]).T
U = worldToPlane(W,H,K)



plot.cal(T,R)
# 'Y' NEGATIVA PARA PODER VERLO EN PYPLOT (Y EN REFERENCIAL DE CAMARA ES NEGATIVA)
plot.draw_point(PW[0][0],-PW[1][0],PW[2][0],'b')
plot.draw_space()
plot.show_plot()
