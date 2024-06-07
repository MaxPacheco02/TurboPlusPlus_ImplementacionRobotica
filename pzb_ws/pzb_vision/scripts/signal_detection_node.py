#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import os

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pzb_msgs.msg import Signal

from ultralytics import YOLO

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

def lowest_detected(detections):
    lowest_id = 0
    for i in range(1,len(detections)):
        if detections[i][2] < detections[lowest_id][2]:
            lowest_id = i
    return lowest_id

class SignalDetection(Node):
    def __init__(self):
        super().__init__('signal_detection')
        self.signal_pub = self.create_publisher(Signal, '/signal_detected', 10)
        self.frame_sub = self.create_subscription(Image, '/signal_frame', self.subscriber_callback, 10)
        self.get_logger().info('signal_detection Initialized')

        self.bridge = CvBridge()
        self.signal = Signal()

        self.yolo_file_ = os.path.join(
            get_package_share_directory('pzb_vision'),
            'config',
            'yolo_signals.pt'
        )

        self.yolo = YOLO(self.yolo_file_)  # pretrained YOLOv8n model

        self.signals =	{
            "stop": Signal.STOP,
            "left": Signal.LEFT,
            "circle": Signal.CIRCLE,
            "slow": Signal.SLOW,
        }
        
    def subscriber_callback(self, IMG):

        img = self.bridge.imgmsg_to_cv2(IMG,desired_encoding='passthrough')
        w = img.shape[1] * 5
        h = img.shape[0] * 5
        img = cv2.resize(img,(w,h))
        img_og = img.copy()

        results = self.yolo(img, stream=True)
        detections = []
        for result in results:
            for box in result.boxes:
                cv2.rectangle(img, (int(box.xyxy[0][0]), int(box.xyxy[0][1])),
                            (int(box.xyxy[0][2]), int(box.xyxy[0][3])), (255, 0, 0), 5)
                cv2.putText(img, f"{result.names[int(box.cls[0])]}",
                            (int(box.xyxy[0][0]), int(box.xyxy[0][1]) - 10),
                            cv2.FONT_HERSHEY_PLAIN, 5, (255, 0, 0), 5)
                detections.append([result.names[int(box.cls[0])], int(box.xyxy[0][0]), int(box.xyxy[0][1])])

        detected_signal = ''
        if len(detections) > 0:
            detected_signal = detections[lowest_detected(detections)][0]
        
        self.signal.signal = -1
        if detected_signal in self.signals:
            self.signal.signal = self.signals[detected_signal]
        self.signal_pub.publish(self.signal)

        # # Viewing
        # frames = cv2_conc([img_og, img])
        # cv2.imshow('frames', frames)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    l_d = SignalDetection()
    rclpy.spin(l_d)
    l_d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
