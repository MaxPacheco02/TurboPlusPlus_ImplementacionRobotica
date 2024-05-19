import cv2
import glob
import numpy as np
import math
from Draw_ax import Draw

def printParams(ret, mtx, dist, rvecs, tvecs, dim):
    print("\nmtx ",dim," : \n",mtx)
    print("\ndist ",dim," : \n", dist)
    print("\nrvecs ",dim," : \n", rvecs)
    print("\ntvecs ",dim," : \n", tvecs)
    print("\n-----------------------------------")


def undist(dist,mtx):
    img = cv2.imread('Pic.png')
    w = 1280
    h = 720
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite('calibresult2.png', dst)
    return newcameramtx


def worldToPlane(W,Hp,K):
    # CONSTRUIR MATRIZ INTRINSECA
    M = np.concatenate((K,np.array([[0,0,0]]).T), axis=1)
    # DE REFERENCIAL DE MUNDO A REFERENCIAL DE CAMARA
    C = Hp @ W
    print("\nC_p = \n",C)
    # DE REFERENCIAL DE CAMARA A PLANO DE IMAGEN
    V = M @ C
    # PUNTO EN PIXELES EN PLANO DE IMAGEN (u,v) -> (X,Y)
    u = V[0]/V[2]  # Xc / Zc
    v = V[1]/V[2]  # Yc / Zc
    return [round(np.ravel(u)[0],0),round(np.ravel(v)[0],0)]


def planeToWorld(R,Tp,K,PUV,Zc):
    # DE PLANO DE IMAGEN A COORDENADAS NORMALIZADAS DE REFERENCIAL DE CAMARA
    C_norm = np.linalg.inv(K) @ PUV
    print("\nC_norm = \n",C_norm)
    # DE COORDENADAS NORMALIZADAS A REFERENCIAL DE CAMARA 
    C = Zc * C_norm
    print("\nC = \n",C)
    # AGREGRAR 1 A CORDS DE REFERENCIAL DE CAMARA [Xc;Yc;Zc;1]
    C = np.concatenate((C,[[1]]), axis = 0)
    print("\nC = \n",C)
    # CONSTRUIR INVERSA DE LA MATRIZ HOMOGENEA 
    H_inv = np.concatenate((np.concatenate((R.T,(-R.T)@Tp), axis=1), [[0,0,0,1]]), axis=0)
    print("\nH_inv = \n",H_inv)
    # DE REFERENCIAL DE CAMARA A REFERENCIAL DE MUNDO
    PW = H_inv @ C
    PW = PW * -1
    print("\nPW = \n",PW)
    return PW


# CALLBACK DE CLICK DE MOUSE
def select_point(event, x, y, flags, param):
    # EVENTO DE CLICK IZQUIERDO DE MOUSE
    if event == cv2.EVENT_LBUTTONDOWN:
        global PUV 
        # OBTENER COORDENADAS DE PIXEL SELECCIONADO
        arr = np.array([[x, y]], np.float32)
        print("\n(",x," , ",y,")")
        PUV[0][0] = arr[0][0]
        PUV[1][0] = arr[0][1]
        PUV[2][0] = 1
        cv2.circle(param, (x, y), 5, (0, 255, 0), -1) 
        cv2.imshow("Imagen", param)


# SELECCIONAR PUNTO EN IMAGEN
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


def select_point2(event, x, y, flags, param):
    # EVENTO DE CLICK IZQUIERDO DE MOUSE
    if event == cv2.EVENT_LBUTTONDOWN:
        global PBUV 
        # OBTENER COORDENADAS DE PIXEL SELECCIONADO
        arr = np.array([[x, y]], np.float32)
        PBUV = np.append(PBUV,arr,axis=0)
        cv2.circle(param, (x, y), 5, (0, 255, 0), -1) 
        cv2.imshow("Imagen", param)


def box(name):
    IMG = cv2.imread(name)
    cv2.namedWindow("Imagen")
    while True:
        cv2.imshow("Imagen", IMG)
        cv2.setMouseCallback("Imagen", select_point2, IMG)
        k = cv2.waitKey(1)
        if(k==ord('q')):
            break
    cv2.destroyAllWindows()


def boxCal(f,X,A,h,w,Wd,PBUV):
    # LONGITUD EN PIXELES
    w = PBUV[1][0] - PBUV[0][0]
    h = PBUV[1][1] - PBUV[0][1]
    Wpx = math.sqrt( (w)**2 + (h)**2 )
    Wmm = (Wpx * A) / X
    d = (Wd * f) / Wmm
    return d


# INICIALIZAR OBJETO DE PLOT
plot = Draw()

# PARAMETROS DE CAMERA DE LAPTOP 720p (ESTIMADO)
f = 3
X = 1280
A = 3.6

# DIMENSIONES DE OBJETO (mm)
h = 90
w = 160
Wd = 85
PBUV = np.zeros((0,2),np.float32)

# NUMERO DE ESQUINAS EN CHESSBOARD DE CALIBRACION
chessSize = (5,5)

# TAMAÑO DE CADA CUADRO EN MM
chessSq = 31

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# PUNTOS PARA PLANOS DE CALIBRACION
objp = np.zeros((chessSize[0]*chessSize[0],3), np.float32)
objp[:,:2] = np.mgrid[0:chessSize[0],0:chessSize[1]].T.reshape(-1,2)

# ASIGNAR MEDIDAS REALES A OBJP (MM)
objp = objp * chessSq

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('./chessboard/*.png')

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

# CALIBRACION DE CAMARA
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, None)

#VECTOR DE TRASLACION
T = tvecs[4]

# VECTOR DE TRASLACION CON X POSITIVA
Tp = np.copy(T)
# 'X' EN REFERENCIAL DE MUNDO DE IMAGEN[4] ES NEGATIVA (PARA REPRESENTARLO EN LA GRAFICA)
T[0][0] = T[0][0] * -1
# MATRIZ DE ROTACION MUNDO A REFERENCIAL DE CAMARA
R, _ = cv2.Rodrigues(rvecs[4])

print("\nR = \n",R)
print("\nrvecs[4] = \n",rvecs[4])
print("\ntvecs[4] = \n",tvecs[4])

# MATRIZ PARA PUNTO EN PLANO DE IMAGEN
PUV = np.zeros((3,1))

# CONSTRUIR MATRIZ DE VALORES EXTRINSECOS P 4x4 (MATRIZ DE TRANSFORMACION HOMOGENEA)
H = np.concatenate((np.concatenate((R , T), axis=1),[[0,0,0,1]]), axis=0)
Hp = np.concatenate((np.concatenate((R , Tp), axis=1),[[0,0,0,1]]), axis=0)

print("\nH = \n",H)

# MATRIZ DE CALIBRACION K
K = mtx

print("\nK = \n",K)

# MATRIZ DE PROYECCION
# ---------------------
#   P' = M * Pw        ------>  P' = (K * [R T]) * Pw
#   P' * inv(M) = Pw
# ---------------------

# OBTENER PUNTOS EN PLANO DE IMAGEN
track()

# DE PLANO DE IMAGEN A MUNDO 
# DISTANCIA DE PUNTO A LA REFERENCIAL DE CAMARA (Por ahora estimada)
Zc = 450
PW = planeToWorld(R,Tp,K,PUV,Zc)

print("\nPW = \n",PW)

# PUNTO EN EL ESPACIO 3D
W = np.array([[31,0,0,1]]).T
U = worldToPlane(W,Hp,K)

print("\nU = \n",U)


box("./BOOK/BOOK_1.jpg")
print("\nPBUV = \n",PBUV)
d = boxCal(f,X,A,h,w,Wd,PBUV)
print("\nd = \n",d)
print("\nnp = \n",np.array([[PBUV[0][0],PBUV[0][1],1]]).T)

PW1 = planeToWorld(R,Tp,K,np.array([[PBUV[0][0],PBUV[0][1],1]]).T,d)
PW2 = planeToWorld(R,Tp,K,np.array([[PBUV[1][0],PBUV[1][1],1]]).T,d)
plot.draw_point(PW1[0][0],-PW1[1][0],PW1[2][0],'b')
plot.draw_point(PW2[0][0],-PW2[1][0],PW2[2][0],'b')


PBUV = np.zeros((0,2),np.float32)
box("./BOOK/BOOK_2.jpg")
print("\nPBUV = \n",PBUV)
d = boxCal(f,X,A,h,w,Wd,PBUV)
print("\nd = \n",d)
print("\nnp = \n",np.array([[PBUV[0][0],PBUV[0][1],1]]).T)

PW1 = planeToWorld(R,Tp,K,np.array([[PBUV[0][0],PBUV[0][1],1]]).T,d)
PW2 = planeToWorld(R,Tp,K,np.array([[PBUV[1][0],PBUV[1][1],1]]).T,d)
plot.draw_point(PW1[0][0],-PW1[1][0],PW1[2][0],'r')
plot.draw_point(PW2[0][0],-PW2[1][0],PW2[2][0],'r')



PBUV = np.zeros((0,2),np.float32)
box("./BOOK/BOOK_3.jpg")
print("\nPBUV = \n",PBUV)
d = boxCal(f,X,A,h,w,Wd,PBUV)
print("\nd = \n",d)
print("\nnp = \n",np.array([[PBUV[0][0],PBUV[0][1],1]]).T)

PW1 = planeToWorld(R,Tp,K,np.array([[PBUV[0][0],PBUV[0][1],1]]).T,d)
PW2 = planeToWorld(R,Tp,K,np.array([[PBUV[1][0],PBUV[1][1],1]]).T,d)
plot.draw_point(PW1[0][0],-PW1[1][0],PW1[2][0],'r')
plot.draw_point(PW2[0][0],-PW2[1][0],PW2[2][0],'r')



plot.cal(T,R)
# 'Y' NEGATIVA PARA PODER VERLO EN PYPLOT 
plot.draw_point(PW[0][0],-PW[1][0],PW[2][0],'r')
plot.draw_space()
plot.show_plot()
