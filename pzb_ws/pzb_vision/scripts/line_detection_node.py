#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

count = 0

min_thresh = 143
chosen_idx = 0
brightness = -277
contrast = 3.5
low_canny = 50
high_canny = 150

def cv2_conc(imgs):
    frames = imgs[0].copy()
    if len(frames.shape) == 2:
        frames = cv2.cvtColor(frames, cv2.COLOR_GRAY2RGB)

    for i in range(len(imgs)-1):
        tmp = imgs[i+1].copy()
        if len(tmp.shape) == 2:
            tmp = cv2.cvtColor(tmp, cv2.COLOR_GRAY2RGB)
        frames = np.concatenate((frames, tmp), axis=0) 
    return frames

def validate_window(in_p, out_p, lims):
    diff = out_p - in_p
    diff_valid = 1 if (diff < lims[1] and diff > lims[0]) else 0
    return [in_p, out_p, diff_valid]



def pure_ring(arr, ring, lims, n):
    in_p = 0
    out_p = 0
    last_p = 0

    windows = []

    for i in range(len(arr[0])):
        # print(len(arr) * ring // n)
        if arr[(len(arr) * ring // n)][i]:
            if last_p == 0:
                in_p = i
            out_p = i

            if len(windows) > 0 and i == len(arr[0]) - 1 and (out_p != windows[-1][1]):
                windows.append(validate_window(in_p, out_p, lims))

        else:
            if last_p != 0:
                windows.append(validate_window(in_p, out_p, lims))

        last_p = arr[len(arr) * ring // n][i]

    # print(ring, windows)
    return windows

def is_joined_window(w1, w2):
    if max(w1[0], w2[0]) < min(w1[1], w2[1]):
        return True
    return False

def pure_thresh(self,arr):
    window_list = []
    n = 15
    for i in range(n-1):
        window_list.append(pure_ring(arr , i+1 , [90,200], n))

    windows_pure = []
    for windows in window_list:
        for window in windows:
            if window[2]:
                for i in range(len(windows_pure)):
                    if is_joined_window(windows_pure[i], window):
                        windows_pure[i] = [min(windows_pure[i][0], window[0]), max(windows_pure[i][1], window[1]), windows_pure[i][2] + window[2]]
                    else:
                        if window not in windows_pure:
                            windows_pure.append(window)
                if len(windows_pure) == 0:
                    windows_pure.append(window)   

    windows_purest = []
    for i in range(len(windows_pure)):
        purest = i
        for j in range(len(windows_pure)):
            if (i != j and is_joined_window(windows_pure[purest], windows_pure[j]) 
                and (windows_pure[j][2] > windows_pure[purest][2])):
                purest = j
        if windows_pure[purest] not in windows_purest:
            windows_purest.append(windows_pure[purest])

    pure_image = np.copy(arr) * 0
    clusters = 0
    for windows in windows_purest:
        if windows[2] > n * 0.1:
            clusters = clusters+1
            # self.get_logger().info(f'{windows[0]},{windows[1]},{windows[2]}')
            window_offset = len(arr[0]) // 50
            window_offset = len(arr[0]) // 1000
            w_lims = [windows[0] - window_offset, windows[1] + window_offset]
            pure_image[::, w_lims[0]:w_lims[1]] = arr[::, w_lims[0]:w_lims[1]]

    return pure_image, clusters

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
        img = self.bridge.imgmsg_to_cv2(IMG,desired_encoding='passthrough')

        w = img.shape[1] * 5
        h = img.shape[0] * 5
        img = cv2.resize(img,(w,h))

        x_c = int(w/2)
        x_t = int(w*.3)
        img = img[:: , x_c - x_t : x_c + x_t]

        img_weighted = cv2.addWeighted(img, contrast, np.zeros(img.shape, img.dtype), 0, brightness)
        grayImage = cv2.cvtColor(img_weighted, cv2.COLOR_BGR2GRAY)

        ret,thresh = cv2.threshold(grayImage,min_thresh,255,cv2.THRESH_BINARY_INV)

        thresh_tmp, clusters = pure_thresh(self,thresh)
        thresh2 = thresh_tmp.copy()

        col = 0
        colsum = []

        for i in range(len(img[0])):
            for j in range(len(img)):
                col += thresh2[j][i] 
            colsum.append(int(col/j))
            col = 0

        # PORCENTAJE PARA THRESHOLD
        thr_p = 0.8

        # THRESHOLD PARA DETERMINAR LINEA
        threshold = thr_p * max(colsum)

        # print(colsum[2])

        for i in range(len(img[0])):
            for j in range(len(img)):
                if(colsum[i] < threshold):
                    thresh2[j][i] = 0
                else:
                    thresh2[j][i] = 255

        # MOMENTOS (PARA ENCONTRAR CENTRO DE BLOB)
        M = cv2.moments(thresh2)

        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # print("cX: " + str(cX) + "  cY: " + str(cY))
        else:
            cX, cY = 0, 0

        # # Viewing
        # viewingFrame = img.copy()
        # if cX != thresh2.shape[1]//2 - 1 or cY != thresh2.shape[0]//2 - 1:
        #     cv2.circle(viewingFrame,(cX, cY), 10, (0,0,255),-1)

        # frames = cv2_conc([img, img_weighted, thresh, thresh_tmp, thresh2, viewingFrame])
        # cv2.imshow('frames', frames)
        # cv2.waitKey(1)

        self.error.data = cX - thresh2.shape[1]//2
        self.error_pub.publish(self.error)

def main(args=None):
    rclpy.init(args=args)
    l_d = LineDetection()
    rclpy.spin(l_d)
    l_d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
