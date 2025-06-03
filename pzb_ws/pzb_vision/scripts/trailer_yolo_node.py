#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
import math
import numpy as np

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO

from pzb_msgs.srv import IsTrailerOnWatch
from pzb_msgs.msg import TrailerBearing


class TrailerYoloNode(Node):
    def __init__(self):
        super().__init__('trailer_angle_service')

        self.fx = 531.16
        self.cx = 291.21

        model_path = os.path.join(
            get_package_share_directory('pzb_vision'),
            'config',
            'best.pt'
        )
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.bearing_angles_pub_ = self.create_publisher(TrailerBearing, '/yolo_bearing_angles', 10)
        self.latest_image = None

        self.create_subscription(CompressedImage, '/signal_frame', self.image_callback, 10)
        self.srv = self.create_service(IsTrailerOnWatch, 'trailer_on_watch', self.handle_request,
                                       callback_group=ReentrantCallbackGroup())
        self.get_logger().info('TrailerYoloNode node with service interface ready')
        # Initialize the bearing angles message with NaN values
        self.bearing_angles_ = TrailerBearing(
            diagonal=float('nan'),
            oxidado=float('nan'),
            rojo=float('nan')
        )

        self.latest_detections = {
            "diagonal": None,
            "oxidado": None,
            "rojo": None
        }
    
    def update_bearing_msg(self, name, bearing):
        if name == "diagonal":
            self.bearing_angles_.diagonal = bearing
        elif name == "oxidado":
            self.bearing_angles_.oxidado = bearing
        elif name == "rojo":
            self.bearing_angles_.rojo = bearing

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        results = self.model(img, stream=True)

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                class_id = int(box.cls[0])
                self.latest_image = result.names[class_id]

                if conf > 0.65:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2
                    self.update_bearing_msg(self.latest_image, -math.atan((center_x - self.cx) / self.fx))
                    self.latest_detections[self.latest_image] = self.get_clock().now()
            
        
        current_time = self.get_clock().now()
        for name, detection_time in list(self.latest_detections.items()):
            if detection_time is not None and (current_time - detection_time).nanoseconds / 1e9 > 0.5:
                self.update_bearing_msg(name, float('nan'))
                self.latest_detections[name] = None

                
        self.bearing_angles_pub_.publish(self.bearing_angles_)

    def handle_request(self, request, response):
        response.is_on_watch = True
        if self.latest_detections[request.trailer_type] is None:
            response.is_on_watch = False

        return response
        
def main(args=None):
    rclpy.init(args=args)
    node = TrailerYoloNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
