#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

count = 0

class LineDetection(Node):
    def __init__(self):
        super().__init__('line_detection')
        #Create Publishers
        self.error_pub = self.create_publisher(Int32, '/error', 10)
        #Create Subscribers 
        self.frame_sub = self.create_subscription(Image, '/floor_frame', self.subscriber_callback, 10)
        self.get_logger().info('line_detection Initialized')

        self.bridge = CvBridge()
        self.error = Int32()
        
        
    def subscriber_callback(self, IMG):
        global count

        IMG_N = self.bridge.imgmsg_to_cv2(IMG,desired_encoding='passthrough')

        Dim = IMG_N.shape[:2]

        # CENTRO DE IMAGEN EN X
        x_c = int(Dim[1]*0.5)

        # PORCENTAGE A LOS LADOS X
        x_t = int(Dim[1]*1. / 2)

        brightness = 10
        contrast = 1

        # RECORTAR IMAGEN
        frame = IMG_N[:: , x_c - x_t : x_c + x_t]

        # DIMENSIONES DE IMAGEN RECORTADA
        Dim_f = frame.shape[:2]

        # CENTRO DE IMAGEN NUEVA X
        xn_c = int(Dim_f[1]*0.5)

        frame = cv2.addWeighted(frame, contrast, np.zeros(frame.shape, frame.dtype), 0, brightness)

        grayImage = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(grayImage,150,255,cv2.THRESH_BINARY_INV)

        col = 0
        colsum = []

        for i in range(0, Dim_f[1]):
            for j in range(0, Dim_f[0]):
                col += thresh[j][i] 
            colsum.append(int(col/j))
            col = 0

        # PORCENTAJE PARA THRESHOLD
        thr_p = 0.8

        # THRESHOLD PARA DETERMINAR LINEA
        threshold = thr_p * max(colsum)

        # print(colsum[2])

        for i in range(0, Dim_f[1]):
            for j in range(0, Dim_f[0]):
                if(colsum[i] < threshold):
                    thresh[j][i] = 0
                else:
                    thresh[j][i] = 255

        # MOMENTOS (PARA ENCONTRAR CENTRO DE BLOB)
        M = cv2.moments(thresh)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # print("cX: " + str(cX) + "  cY: " + str(cY))
        else:
            cX, cY = 0, 0

        self.error.data = cX - xn_c 
        self.error_pub.publish(self.error)

        viewingFrame = grayImage.copy()
        cv2.circle(viewingFrame,(cX, cY), 10, (0,0,255),-1)
        cv2.imshow('frame',viewingFrame)
        key = cv2.waitKey(1)
        # To save rosbag to images
        count_n = 10
        if count%count_n == 0:
            cv2.imwrite("/home/max/irs/src/TurboPlusPlus_ImplementacionRobotica/pzb_ws/pzb_vision/scripts/img"+str(int(count/count_n))+".png", IMG_N)
        count+=1

def main(args=None):
    rclpy.init(args=args)
    l_d = LineDetection()
    rclpy.spin(l_d)
    l_d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
