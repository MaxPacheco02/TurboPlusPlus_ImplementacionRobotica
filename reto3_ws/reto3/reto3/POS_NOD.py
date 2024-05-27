#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np   
import cv2

from .POS_CAL import CAM
from .POS_FU import POS
from .POS_IM import POINTS

class ObjPos(Node):

    def __init__(self):
        super().__init__('ObjPos')
        self.get_logger().info('TEST PYTHON')

        # CREAR PUBLISHER PARA POSICION DE REFERENCIAL DE CAMARA (REFERENCIAL DE MUNDO SIEMPRE ESTARA EN [0,0])
        self.refPos_publisher = self.create_publisher(Float32MultiArray, 'ref_pos', 10)

        # CREAR PUBLISHER PARA POSICION DE OBJETO EN EL MUNDO
        self.objPos_publisher = self.create_publisher(Float32MultiArray, 'obj_pos', 10)

        # CALIBRACION DE CAMARA Y MASCARA
        self.cam = CAM()
        self.cam.cameraCalibrate()
        self.cam.colorCal()

        # PARA OBTENCION DE PUNTOS EN PLANO DE IMAGEN
        self.point = POINTS()

        # POSICION DE MUNDO Y CAMARA 
        self.pos = POS(self.cam,0)
        self.Rc, self.Pc = self.pos.worldCal()

        # POSICION DE OBJETO EN EL MUNDO
        self.PB1 = 0
        self.PB2 = 0
        
        # POSICION DE OBJETO
        self.OBJ = Float32MultiArray()
        # POSICION DE CAMARA
        self.POS = Float32MultiArray()

        self.show()


    def show(self):

        IMG = cv2.VideoCapture(0)
        IMG .set(cv2.CAP_PROP_FRAME_WIDTH,1280)
        IMG .set(cv2.CAP_PROP_FRAME_HEIGHT,720)

        while True:
            ret, frame = IMG.read()
            # CONVERTIR A HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # OBTENER MASCARA PARA ENCONTRAR CONTORNOS
            mask = self.cam.procMask(hsv)

            # ENCONTRAR PUNTOS DE OBJETO Y SU DISTANCIA A LA CAMARA
            img, d, PUV = self.point.boxCal(mask, frame)

            # CALCULAR POSICION DE PUNTOS DE OBJETO EN EL MUNDO
            self.PB1 = self.pos.objCal(np.array([[PUV[0][0],PUV[0][1],1]]).T,d)
            self.PB2 = self.pos.objCal(np.array([[PUV[1][0],PUV[1][1],1]]).T,d)

            #Negative X
            #self.PB1[0][0] *= -1
            #self.PB2[0][0] *= -1
            #Negative Y
            #self.PB1[1][0] *= -1
            #self.PB2[1][0] *= -1

            # PUBLICAR LISTA DE POSICIONES
            self.OBJ.data = np.ravel(self.PB1[:-1]).tolist() + np.ravel(self.PB2[:-1]).tolist()
            self.objPos_publisher.publish(self.OBJ)

            # PUBLICAR LISTA DE POSICIONES DE REFERENCIALES 
            self.POS.data = self.Rc[:,0].tolist() + self.Rc[:,1].tolist() + \
                            self.Rc[:,2].tolist() + np.ravel(self.Pc[:-1]).tolist()
            self.refPos_publisher.publish(self.POS)

            # Midpoint of object (changing Axes for Xarm)
            X_OFFSET = 0.3
            Y_OFFSET = 0.3
            Z_OFFSET = 0.0
            scale = 0.0027

            oy = (self.PB1[0] + self.PB2[0])/2 * scale + Y_OFFSET # y -> X
            oz = -1*(self.PB1[1] + self.PB2[1])/2 * scale + Z_OFFSET # z -> Y 
            ox = (self.PB1[2] + self.PB2[2])/2 * scale + X_OFFSET # x -> Z

            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(img, 'x: ' + str(ox), (50,50), font, 0.6, (0,0,255), 2, cv2.LINE_4)
            cv2.putText(img, 'y: ' +str(oy), (50,75), font, 0.6, (0,255,0), 2, cv2.LINE_4)
            cv2.putText(img, 'z: ' +str(oz), (50,100), font, 0.6, (255,0,0), 2, cv2.LINE_4)

            cv2.imshow("Imagen",img)

            k = cv2.waitKey(1)
            if(k == ord('q')):
                break255
        IMG.release()
        cv2.destroyAllWindows()
