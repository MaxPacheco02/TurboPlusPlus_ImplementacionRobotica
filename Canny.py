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
            elif(img[i][j] >= High):
                img[i][j] = 255
            # else:
            #     img[i][j] = 255
    return img


if __name__ == '__main__':
    High = 30
    Low = 10
    
    IMG = cv.imread('./image.png')
    #IMG = cv.resize(IMG, (0, 0), fx = 0.1, fy = 0.1)
    H, W = IMG.shape[:2]
    # PASAR DE RGB A GRAYSCALE
    # En RGB cada pixel es un vector de tres valores
    # En gray scale cada pixel tiene un solo valor de intensidad
    IMG_G = cv.cvtColor(IMG, cv.COLOR_BGR2GRAY)
    
    mask = cv.Canny(IMG,High,Low)
    
    GAUSS_K = np.array([[2,4,  5, 4,2],
                        [4,9, 12, 9,4],
                        [5,12,15,12,5],
                        [4,9, 12, 9,4],
                        [2,4,  5, 4,2]], np.float32)/159
    
    GAUSS = cv.filter2D(IMG_G,-1,GAUSS_K)
    
    Gx =  np.array([[-1, 0, 1],
                    [-2, 0, 2],
                    [-1, 0, 1]], np.float32)
    
    Gy =  np.array([[-1,-2,-1],
                    [ 0, 0, 0],
                    [ 1, 2, 1]], np.float32)
                    
    Mx = cv.filter2D(GAUSS,-1,Gx)
    My = cv.filter2D(GAUSS,-1,Gy)
    
    SOBEL = cv.filter2D(Mx,-1,Gy)
    
    # GRADIANTE
    G = np.hypot(Mx, My)
    # DIRECCION 
    theta = np.arctan2(My, Mx)
    
    # Solo nos importa la dirección del gradiente
    NON_MAX = NonMaxSup(G, theta, H, W)
    HYST = Hysteresis(NON_MAX, Low, High, H, W)
    
    cv.imwrite("test.png", HYST)
    #cv.namedWindow("HYST")
    #cv.imshow("H", IMG_G)
    #cv.imshow("HYST", GAUSS)
    #cv.waitKey(0)
    #cv.destroyAllWindows()


    #cv.namedWindow("NON_MAX")
    #cv.imshow("Image", IMG)
    #cv.imshow("SOBEL", SOBEL)
    #cv.imshow("NON_MAX", NON_MAX)
    #cv.imshow("Canny", mask)
    #cv.waitKey(0)
    
    #cv.destroyAllWindows()
