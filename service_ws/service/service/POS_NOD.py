#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np   
import cv2

from .POS_CAL import CAM
from .POS_FU import POS
from .POS_IM import POINTS
from .POS_ARU import ARU

class ObjPos(Node):

    def __init__(self):
        super().__init__('ObjPos')
        self.get_logger().info('TEST PYTHON')

        # CREAR PUBLISHER PARA POSICION DE REFERENCIAL DE CAMARA (REFERENCIAL DE MUNDO SIEMPRE ESTARA EN [0,0])
        self.refPos_publisher = self.create_publisher(Float32MultiArray, 'ref_pos', 10)

        # CREAR PUBLISHER PARA POSICION DE OBJETO EN EL MUNDO
        self.objPos_publisher = self.create_publisher(Float32MultiArray, 'obj_pos', 10)

        # CALIBRACION DE CAMARA
        self.cam = CAM()
        self.cam.cameraCalibrate()

        # PARA DETECCION Y TRACKING DE ARUCO
        self.aru = ARU()

        # PARA OBTENCION DE PUNTOS EN PLANO DE IMAGEN
        self.point = POINTS(self.aru.markerLength)

        # POSICION DE MUNDO Y CAMARA 
        self.pos = POS(self.cam,2)
        self.Rc, self.Pc = self.pos.worldCal()

        # POSICION PUNTOS OBJETO EN EL MUNDO
        self.PB1 = 0
        self.PB2 = 0

        # POSICION DE ARUCO EN EL MUNDO
        self.aruPOS = np.zeros((4,1),np.float32)

        # ORIENTACION DE ARUCO EN EL MUNDO
        self.aruORI = np.zeros((3,3),np.float32)

        # POSICION DE OBJETO
        self.OBJ = Float32MultiArray()
        # POSICION DE CAMARA
        self.POS = Float32MultiArray()

        self.show()


    def show(self):

        IMG = cv2.VideoCapture(0)
        IMG.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        IMG.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)

        temp = []
        ID = []

        while True:
            ret, frame = IMG.read()
            
            if(ret is None):
                continue

            # ARU DISTANCIA
            frame, aru_ROT, aru_TRA, ID  = self.aru.pose_estimation(frame,self.cam.CamMat,self.cam.CamDis)
            print("\nID : ",ID ) 
            if(len(ID) > 0):
                temp.clear()
                for i in range(0,len(ID)):

                    # CALCULAR ORIENTACION DE ARUCO EN EL MUNDO
                    self.aruORI, self.aruPOS = self.pos.aruOriToWorld(aru_ROT[i],aru_TRA[i])
                    # print("\naru_TRA[i] : ",aru_TRA[i])

                    temp = temp + self.aruORI[:,0].tolist() + self.aruORI[:,1].tolist() + \
                            self.aruORI[:,2].tolist() + np.ravel(self.aruPOS[:-1]).tolist() + \
                            [float(ID[i][0])]


            # PUBLICAR LISTA DE POSICIONES
            self.OBJ.data = temp
            self.objPos_publisher.publish(self.OBJ)
            temp = []

            # PUBLICAR LISTA DE POSICIONES DE REFERENCIAL DE CAMARA
            self.POS.data = self.Rc[:,0].tolist() + self.Rc[:,1].tolist() + \
                            self.Rc[:,2].tolist() + np.ravel(self.Pc[:-1]).tolist()
            self.refPos_publisher.publish(self.POS)

            cv2.imshow("Imagen",frame)

            k = cv2.waitKey(1)

            # ******* FOR DEMO PURPOSES ONLY ********* 
            if(k == ord('q')):
                break

        IMG.release()
        cv2.destroyAllWindows()