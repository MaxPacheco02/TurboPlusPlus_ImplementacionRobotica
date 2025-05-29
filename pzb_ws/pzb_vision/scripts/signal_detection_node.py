#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import CompressedImage, LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
from tf_transformations import quaternion_from_euler

class TrailerPoseEstimator(Node):
    def __init__(self):
        super().__init__('trailer_pose_estimator')

        # Camera intrinsics
        self.fx = 531.16
        self.cx = 291.21

        model_path = os.path.join(
            get_package_share_directory('pzb_vision'),
            'config',
            'best.pt'
        )
        self.model = YOLO(model_path)
        self.trailer_classes = ["rojo", "diagonal", "oxidado"]
        self.bridge = CvBridge()

        # Subscriptions
        self.create_subscription(CompressedImage, '/signal_frame', self.image_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # Marker Publishers
        self.marker_center_pub = self.create_publisher(Marker, '/trailer_center_marker', 10)
        self.marker_arrow_pub = self.create_publisher(Marker, '/trailer_alignment_marker', 10)
        self.marker_left_pub = self.create_publisher(Marker, '/trailer_left_marker', 10)
        self.marker_right_pub = self.create_publisher(Marker, '/trailer_right_marker', 10)

        # Pose Publisher
        self.pose_pub = self.create_publisher(PoseStamped, '/desired_trailer_pose', 10)

        # State
        self.trailer_center_x = None
        self.image_width = None
        self.detected_class = None
        self.center_range_xy = None

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.image_width = img.shape[1]
        results = self.model(img, stream=True)

        for result in results:
            for box in result.boxes:
                class_name = result.names[int(box.cls[0])]
                if class_name in self.trailer_classes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    self.trailer_center_x = (x1 + x2) // 2
                    self.detected_class = class_name
                    self.publish_center_marker()
                    return  

    def laser_callback(self, msg):
        if self.trailer_center_x is None or self.image_width is None:
            return

        # Convert pixel to angle
        u = self.trailer_center_x
        theta = math.atan((u - self.cx) / self.fx)
        center_idx = int((theta - msg.angle_min) / msg.angle_increment)

        self.get_logger().info(
            f"Pixel {u} → θ = {math.degrees(theta):.2f}° → LaserScan index {center_idx}"
        )

        offset = int(math.radians(1) / msg.angle_increment)
        indices = [center_idx - offset, center_idx, center_idx + offset]

        points = []
        for idx in indices:
            if 0 <= idx < len(msg.ranges):
                r = msg.ranges[idx]
                if msg.range_min < r < msg.range_max:
                    angle = msg.angle_min + idx * msg.angle_increment
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append((x, y))

        if len(points) != 3:
            return

        # Normal vector from P1 → P3
        dx = points[2][0] - points[0][0]
        dy = points[2][1] - points[0][1]
        nx = -dy
        ny = dx

        # Flip normal to point toward robot
        vx = -points[1][0]
        vy = -points[1][1]
        dot = nx * vx + ny * vy
        if dot < 0:
            nx *= -1
            ny *= -1

        # Normalize and compute desired position 15cm in front of trailer
        length = math.hypot(nx, ny)
        nx /= length
        ny /= length

        x, y = points[1]
        desired_x = x - 0.15 * nx
        desired_y = y - 0.15 * ny
        self.center_range_xy = (desired_x, desired_y)

        normal_angle = math.atan2(ny, nx) + math.pi
        quat = quaternion_from_euler(0, 0, normal_angle)

        # Final corrected markers and pose
        self.publish_arrow_marker(desired_x, desired_y, quat)
        self.publish_pose(desired_x, desired_y, quat)
        self.publish_point_marker(points[0], self.marker_left_pub, "left", 1.0, 0.0, 0.0)
        self.publish_point_marker(points[2], self.marker_right_pub, "right", 0.0, 1.0, 0.0)


    def publish_center_marker(self):
        if self.center_range_xy is None:
            return

        x, y = self.center_range_xy
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trailer_center"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2

        if self.detected_class == "rojo":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif self.detected_class == "diagonal":
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif self.detected_class == "oxidado":
            marker.color.r = 0.6
            marker.color.g = 0.3
            marker.color.b = 0.1
        else:
            marker.color.r = marker.color.g = marker.color.b = 1.0

        marker.color.a = 0.9
        marker.lifetime.sec = 1
        self.marker_center_pub.publish(marker)

    def publish_arrow_marker(self, x, y, quat):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trailer_alignment"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = 0.5
        marker.scale.y = marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        self.marker_arrow_pub.publish(marker)

    def publish_pose(self, x, y, quat):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.pose_pub.publish(pose)

    def publish_point_marker(self, point, publisher, ns, r, g, b):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"trailer_{ns}_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TrailerPoseEstimator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
