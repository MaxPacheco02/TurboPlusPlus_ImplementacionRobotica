import numpy as np
import cv2 as cv


def NonMaxSup(img, theta, H, W):
    NON_MAX = np.empty((H,W))
    deg = (theta * 180.0)/np.pi
    izq = 0
    der = 0
    for i in range(0, H):
        for j in range(0, W):
            
            try:
                #NON_MAX[i][j] = 0
                if(deg[i][j] <= 0.0):
                    der = img[i][j+1]
                    izq = img[i][j-1]
                elif(0 < deg[i][j] <= 45.0):
                    der = img[i-1][j+1]
                    izq = img[i+1][j-1]
                elif(45.0 < deg[i][j] <= 90.0):
                    der = img[i+1][j] # PIXEL ABAJO
                    izq = img[i-1][j] # PIXEL ARRIBA
                elif(90.0 < deg[i][j] <= 135.0):
                    der = img[i+1][j+1]
                    izq = img[i-1][j-1]
                
                if(img[i][j] >= der and img[i][j] >= izq):
                    NON_MAX[i][j] = img[i][j]
                else:
                    NON_MAX[i][j] = 0
            
            except IndexError as e: 
                pass
    return NON_MAX


def Hysteresis(img, Low, High, H, W):
    for i in range(0,H):
        for j in range(0,W):
            if(Low < img[i][j] < High):
                try:
                    if((img[i][j+1] > High) or (img[i][j-1] > High) or (img[i-1][j+1] > High) or 
                        (img[i+1][j-1] > High) or (img[i+1][j] > High) or (img[i-1][j] > High) or 
                        (img[i+1][j+1] > High) or (img[i-1][j-1] > High)):
                        img[i][j] = 255
                    else:
                        img[i][j] = 0
                except IndexError as e: 
                    pass
            elif(img[i][j] > High):
                img[i][j] = 255
            else:
                img[i][j] = 0
    return img

High = 30
Low = 10

GAUSS_K = np.array([[2,4,  5, 4,2],
                    [4,9, 12, 9,4],
                    [5,12,15,12,5],
                    [4,9, 12, 9,4],
                    [2,4,  5, 4,2]], np.float32)/159

Gx =  np.array([[-1, 0, 1],
                [-2, 0, 2],
                [-1, 0, 1]], np.float32)

Gy =  np.array([[-1,-2,-1],
                [ 0, 0, 0],
                [ 1, 2, 1]], np.float32)

IMG = cv.VideoCapture(0)
W = IMG.get(cv.CAP_PROP_FRAME_WIDTH)
H = IMG.get(cv.CAP_PROP_FRAME_HEIGHT)
cv.namedWindow("Resized_Window") 

while True:
    ret, frame = IMG.read()
    IMG_E = cv.Canny(frame, Low, High)
    IMG_G = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    GAUSS = cv.GaussianBlur(IMG_G,(15,15),15) #cv.filter2D(IMG_G,-1,GAUSS_K)
    Mx = cv.filter2D(GAUSS,-1,Gx)
    My = cv.filter2D(GAUSS,-1,Gy)
    #SOBEL = cv.filter2D(Mx,-1,Gy)
    # GRADIANTE
    G = np.hypot(Mx, My)
    # DIRECCION 
    theta = np.arctan2(My, Mx)

    # Solo nos importa la dirección del gradiente
    NON_MAX = NonMaxSup(G, theta, int(H), int(W))
    
    HYST = Hysteresis(NON_MAX, Low, High, int(H), int(W))

    cv.imshow("Resized_Window", HYST) 
    k = cv.waitKey(1)
    if(k==ord('q')):
        break
cv.destroyAllWindows()