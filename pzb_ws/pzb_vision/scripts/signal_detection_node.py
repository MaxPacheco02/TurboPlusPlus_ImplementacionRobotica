#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import os
import math

from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO


class TrailerAnglePublisher(Node):
    def __init__(self):
        super().__init__('trailer_angle_publisher')

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

        self.create_subscription(CompressedImage, '/signal_frame', self.image_callback, 10)
        self.angle_pub = self.create_publisher(Float64, '/yolo_bearing_angle', 10)
        self.marker_pub = self.create_publisher(Marker, '/trailer_marker', 10)
        self.path_pub = self.create_publisher(Path, '/trailer_path', 10)

        self.get_logger().info('TrailerAnglePublisher node initialized')

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        results = self.model(img, stream=True)

        found_trailer = False

        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                class_id = int(box.cls[0])
                class_name = result.names[class_id]

                if class_name in self.trailer_classes and conf > 0.65:
                    found_trailer = True

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    center_x = (x1 + x2) // 2
                    theta = -math.atan((center_x - self.cx) / self.fx)

                    self.get_logger().info(
                        f"[{class_name}] Detected with conf = {conf:.2f}, θ = {math.degrees(theta):.2f}°"
                    )

                    # Publish angle
                    self.angle_pub.publish(Float64(data=theta))

                    # Place trailer 20 cm forward in X, angle offset in Y
                    x = -0.5
                    y = math.tan(theta) * x
                    z = 0.0  # flat plane for top-down visualization

                    color = self.get_color_for_class(class_name)

                    # Sphere Marker
                    sphere = Marker()
                    sphere.header.frame_id = "base_axis"
                    sphere.header.stamp = self.get_clock().now().to_msg()
                    sphere.ns = "trailer_center"
                    sphere.id = 0
                    sphere.type = Marker.SPHERE
                    sphere.action = Marker.ADD
                    sphere.pose.position.x = x
                    sphere.pose.position.y = y
                    sphere.pose.position.z = z
                    sphere.pose.orientation.w = 1.0
                    sphere.scale.x = 0.1
                    sphere.scale.y = 0.1
                    sphere.scale.z = 0.1
                    sphere.color = color
                    self.marker_pub.publish(sphere)

                    # Arrow Marker
                    arrow = Marker()
                    arrow.header.frame_id = "base_axis"
                    arrow.header.stamp = self.get_clock().now().to_msg()
                    arrow.ns = "trailer_arrow"
                    arrow.id = 1
                    arrow.type = Marker.ARROW
                    arrow.action = Marker.ADD
                    arrow.pose.position.x = 0.0
                    arrow.pose.position.y = 0.0
                    arrow.pose.position.z = z
                    arrow.pose.orientation.z = math.sin(theta / 2)
                    arrow.pose.orientation.w = math.cos(theta / 2)
                    arrow.scale.x = 0.5
                    arrow.scale.y = 0.05
                    arrow.scale.z = 0.05
                    arrow.color = color
                    self.marker_pub.publish(arrow)

                    # Path from origin to trailer
                    path_msg = Path()
                    path_msg.header.frame_id = "base_axis"
                    path_msg.header.stamp = self.get_clock().now().to_msg()

                    start_pose = PoseStamped()
                    start_pose.header = path_msg.header
                    start_pose.pose.position.x = 0.0
                    start_pose.pose.position.y = 0.0
                    start_pose.pose.position.z = z
                    start_pose.pose.orientation.w = 1.0

                    end_pose = PoseStamped()
                    end_pose.header = path_msg.header
                    end_pose.pose.position.x = x
                    end_pose.pose.position.y = y
                    end_pose.pose.position.z = z
                    end_pose.pose.orientation.w = 1.0

                    path_msg.poses = [start_pose, end_pose]
                    self.path_pub.publish(path_msg)

                    return  # only process first valid detection

        if not found_trailer:
            self.remove_marker("trailer_center", 0)
            self.remove_marker("trailer_arrow", 1)
            self.clear_path()

    def remove_marker(self, ns, marker_id):
        marker = Marker()
        marker.header.frame_id = "base_axis"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    def clear_path(self):
        path = Path()
        path.header.frame_id = "base_axis"
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = []
        self.path_pub.publish(path)

    def get_color_for_class(self, class_name):
        color = Marker().color
        color.a = 1.0
        if class_name == "rojo":
            color.r = 1.0
            color.g = 0.0
            color.b = 0.0
        elif class_name == "oxidado":
            color.r = 0.6
            color.g = 0.3
            color.b = 0.0
        elif class_name == "diagonal":
            color.r = 1.0
            color.g = 1.0
            color.b = 0.0
        else:
            color.r = color.g = color.b = 1.0  # fallback
        return color


def main(args=None):
    rclpy.init(args=args)
    node = TrailerAnglePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
