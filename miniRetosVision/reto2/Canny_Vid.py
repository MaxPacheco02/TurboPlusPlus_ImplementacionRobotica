import numpy as np
import cv2 as cv

# NON MAXIMUM SUPRESSION
def NonMaxSup(img, theta, H, W):
    # MATRIZ PARA PIXELES PROCESADOS
    global NON_MAX 
    # CONVERTIR ANGULOS A GRADOS
    deg = (theta * 180.0)/np.pi
    # PARA GUARDAR VALOR DE INTENSIDAD DE PIXELES
    izq = 0
    der = 0
    for i in range(0, H):
        for j in range(0, W):
            try:
                # REVISAR IMTENSIDAD DE PIXELES ALREDEDOR DE PIXEL(i,j)
                #
                #                    90°
                # 135° (i-1,j-1) | (i-1,j) | (i-1,j+1) 45°
                #      -------------------------------
                #      (i , j-1) | (i , j) | (i , j+1) 0°
                #      -------------------------------
                #      (i+1,j-1) | (i+1,j) | (i+1,j+1) 
                #
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
                
                # REVISAR SI PIXEL (i,j) TIENE MAS INTENSIDAD
                # QUE ALREDEDOR
                if(img[i][j] >= der and img[i][j] >= izq):
                    NON_MAX[i][j] = img[i][j]
                else:
                    NON_MAX[i][j] = 0
            
            except IndexError as e: 
                continue
    
    return NON_MAX

# HISTERESIS
def Hysteresis(img, Low, High, H, W):
    for i in range(0,H):
        for j in range(0,W):
            # REVISAR SI INTENSIDAD DE PIXEL (i,j) SE ENCUENTRA ENTRE THRESHOLDS
            if(Low < img[i][j] < High):
                try:
                    # REVISAR PIXELES ALREDEDOR
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


if __name__ == '__main__':
    
    # KERNEL DE GAUSS PARA IMAGE BLUR 
    GAUSS_K = np.array([[2,4,  5, 4,2],
                        [4,9, 12, 9,4],
                        [5,12,15,12,5],
                        [4,9, 12, 9,4],
                        [2,4,  5, 4,2]], np.float32)/159
    
    # OPERADORES SOBEL 
    # Kernels para medir cambio de intensidad 
    # en la imagen de izquierda a derecha y
    # de arriba a abajo.
    Gx =  np.array([[-1, 0, 1],
                    [-2, 0, 2],
                    [-1, 0, 1]], np.float32)
    Gy =  np.array([[-1,-2,-1],
                    [ 0, 0, 0],
                    [ 1, 2, 1]], np.float32)
    
    # CAPTURA DE VIDEO
    IMG = cv.VideoCapture(0)
    # OBTENER DIMENSIONES DE FRAME
    W = IMG.get(cv.CAP_PROP_FRAME_WIDTH)
    H = IMG.get(cv.CAP_PROP_FRAME_HEIGHT)
    cv.namedWindow("Resized_Window") 
    
    NON_MAX = np.empty((int(H),int(W)))
    
    # THRESHOLDS PARA ALGORITMO CANNY
    High = 30
    Low = 10
    
    while True:
        ret, frame = IMG.read()
        
        # CONVERTIR FRAME DE RGB A GRAYSCALE
        # En RGB cada pixel es un vector de tres valores, en cambio
        # en gray scale cada pixel tiene un solo valor de intensidad
        IMG_G = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        # CONVOLUCION DE KERNEL GAUSSIANO CON MATRIZ DE FRAME
        GAUSS = cv.filter2D(IMG_G,-1,GAUSS_K)
        
        # CONVOLUCION IMAGEN DIFUMINADA CON OPERADORES SOBEL
        Mx = cv.filter2D(GAUSS,-1,Gx)
        My = cv.filter2D(GAUSS,-1,Gy)
        
        # MAGNITUD DE GRADIANTES
        G = np.hypot(Mx, My)
        # DIRECCION DE GRADIENTES (ANGULOS)
        theta = np.arctan2(My, Mx)
        
        # APLICAR NON MAXIMUM SUPRESSION
        # Para hacer esto se necesita la magnitud  
        # y la dirección de los gradientes
        NON_MAX = NonMaxSup(G, theta, int(H), int(W))
        
        # APLICAR HISTERESIS
        HYST = Hysteresis(NON_MAX, Low, High, int(H), int(W))
        
        cv.imshow("Resized_Window", HYST) 
        k = cv.waitKey(1)
        if(k==ord('q')):
            break
    
    IMG.release()
    cv.destroyAllWindows()