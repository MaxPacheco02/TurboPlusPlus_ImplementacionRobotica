import cv2
import numpy as np


# IMPRIMIR PARAMETROS DE CAMARA
def printPrams(ret, mtx, dist, rvecs, tvecs, dim):
    global c
    print("\nmtx ",dim," : \n",mtx)
    if(c == 0):
        print("\ndist ",dim," : \n", dist)
        print("\nrvecs ",dim," : \n", rvecs)
        print("\ntvecs ",dim," : \n", tvecs)
        print("\n-----------------------------------")
    pass


# CALLBACK DE CLICK DE MOUSE
def select_point(event, x, y, flags, param):
    # EVENTO DE CLICK IZQUIERDO DE MOUSE 
    if event == cv2.EVENT_LBUTTONDOWN:
        global imagepoints
        global imagepoints2D
        global image2D
        global image
        global c
        # OBTENER COORDENADAS DE PIXEL SELECCIONADO
        arr = np.array([[x, y]], np.float32)
        if(c == 1):
            # AGREGAR PUNTO SELECCIONADO A ARREGLO DE PUNTOS DE IMAGEN
            imagepoints2D = np.append(imagepoints2D, arr, axis=0)
            cv2.circle(image2D, (x, y), 5, (0, 255, 0), -1) 
            cv2.imshow("Image", image2D)
        else:
            # AGREGAR PUNTO SELECCIONADO A ARREGLO DE PUNTOS DE IMAGEN
            imagepoints = np.append(imagepoints, arr, axis=0)
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1) 
            cv2.imshow("Image3D", image)


# WHILE PARA DESPLEGAR IMAGEN Y SELECCIONAR PUNTOS 
def pic_while(image):
    cv2.namedWindow("Image")
    while True:
        cv2.imshow("Image3D", image)
        # CALLBACK DE CLICK DE MOUSE
        cv2.setMouseCallback("Image3D", select_point)
        k = cv2.waitKey(1)
        if(k==ord('q')):
            break
    cv2.destroyAllWindows()

def empty(IMG):
    pass

# CALIBRAR CAMARA 
def cam_cal():
    global imagepoints
    global imagepoints2D
    global image2D
    global image
    global c
    # CARGAR IMAGENES
    image2D = cv2.imread("./CRED_RIG2D.jpeg")
    image = cv2.imread("./CRED_RIG3D.jpeg")
    c = 1
    # PUNTOS EN CREDENCIAL 2D 
    objpoints2D=np.array([[0,0,0],[0,5.4,0],[8.5,5.4,0],
                        [8.5,0,0],[0.6,2.55,0],[0.6,3.55,0],
                        [7.85,3.55,0],[7.85,2.55,0]], np.float32) 
    # PUNTOS EN RIG 3D 
    objpoints=np.array([[8.5,0,5.4],[0,0,5.4],
                        [0,8.5,5.4],[0,8.5,0],
                        [0,5.4,0],[8.5,5.4,0],
                        [8.5,0,0],[0,0,0]], np.float32)
    # ARREGLOS VACIOS PARA PUNTOS EN LA IMAGEN
    imagepoints2D = np.zeros((0,2), np.float32)
    imagepoints = np.zeros((0,2), np.float32)
    # TAMAÑO DE IMAGENES
    img_size2D = image2D.shape[:2]
    img_size = image.shape[:2]
    # ARREGLOS VACIOS PARA FORMAT DE FUNCION CAL
    objp = []
    imgp = []
    
    # SELECCION DE PUNTOS EN IMAGEN 2D
    pic_while(image2D)
    objp.append(objpoints2D)
    imgp.append(imagepoints2D) 
    # CALIBRATECAMERA FUNCTION
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objp, imgp, img_size2D[::-1], None, None, None)
    printPrams(ret, mtx, dist, rvecs, tvecs, '2D')
    
    # LIMPIAR ARREGLOS 
    objp.clear()
    imgp.clear()
    c = 0
    
    # SELECCION DE PUNTOS EN IMAGEN 3D
    pic_while(image)
    objp.append(objpoints)
    imgp.append(imagepoints) 
    # CALIBRATECAMERA FUNCTION
    # Usamos la matriz de camara que obtenimos de calibración con plano 2D como la guess matrix 
    # que requiere la función calibrateCamera() para rigs en 3D.
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objp, imgp, img_size[::-1], mtx, None, None, None, flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    printPrams(ret, mtx, dist, rvecs, tvecs, '3D')


# TRACKING DE LA CREDENCIAL
def cred_track():
    cap = cv2.VideoCapture(0)
    # CREAR TRACKBAR PARA PROPIEDADES DE COLOR
    cv2.namedWindow("Trackbar")
    cv2.resizeWindow("Trackbar",600,300)
    cv2.createTrackbar("hue_min","Trackbar",54,255,empty)   
    cv2.createTrackbar("hue_max","Trackbar",143,255,empty)
    cv2.createTrackbar("sat_min","Trackbar",74,255,empty)
    cv2.createTrackbar("sat_max","Trackbar",255,255,empty)
    cv2.createTrackbar("val_min","Trackbar",154,255,empty)
    cv2.createTrackbar("val_max","Trackbar",217,255,empty)
    
    while True:
        # LEER IMAGEN
        ret, frame = cap.read()
        # CONVERTIR IMAGEN DE ESPACIO DE COLOR BGR A HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # OBTENER VALORES DE PROPIEDADES HSV DE TRACKBAR
        h_min = cv2.getTrackbarPos("hue_min", "Trackbar")
        h_max = cv2.getTrackbarPos("hue_max", "Trackbar")
        s_min = cv2.getTrackbarPos("sat_min", "Trackbar")
        s_max = cv2.getTrackbarPos("sat_max", "Trackbar")
        v_min = cv2.getTrackbarPos("val_min", "Trackbar")
        v_max = cv2.getTrackbarPos("val_max", "Trackbar")
        
        # RANGOS DE VALORES HSV
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        # CREAR MASCARA CON RANGOS HSV
        mask = cv2.inRange(hsv, lower, upper)
        # ENCONTRAR CONTORNOS EN IMAGEN DE MASCARA
        cnts, hie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # FOR PARA DIBUJAR CAJAS
        for c in cnts:
            # AREA FORMADA POR CADA CONTOUR
            area = cv2.contourArea(c)
            if area > 2000:
                # POSICION Y DIMENSIONES DE CONTORNO
                x, y, w, h = cv2.boundingRect(c)
                # DIBUJAR RECTANGULO
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0),2)
                # DIBUJAR CONTORNOS
                cv2.drawContours(frame, c, -1, (0,255,0), 2)
                # DESPLEGAR NOMBRE DE OBJETO
                cv2.putText(frame, "credencial", (x + w + 10, y + h + 10), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0,255,0), 2)
                
        cv2.imshow("Frame", frame)
        #cv2.imshow("Frame", MASK)
        
        k = cv2.waitKey(1)
        if(k==ord('q')):
            break
    cv2.destroyAllWindows()

cam_cal()
cred_track()