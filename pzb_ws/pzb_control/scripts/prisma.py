#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dexterous_msgs.msg import Prism, VoiceCommand
from geometry_msgs.msg import Point, Vector3, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Header

import numpy as np

class Route(Node):

    def __init__(self):
        super().__init__('prisma') 

        self.prisma_publisher = self.create_publisher(Prism, '/prism_coords', 10)

        self.p_cm_publisher = self.create_publisher(Marker, 'p_cm_marker', 10)
        self.p1_publisher = self.create_publisher(Marker, 'p1_marker', 10)
        self.p2_publisher = self.create_publisher(Marker, 'p2_marker', 10)
        self.p3_publisher = self.create_publisher(Marker, 'p3_marker', 10)
        self.p1_c_publisher = self.create_publisher(Marker, 'p1c_marker', 10)
        self.p2_c_publisher = self.create_publisher(Marker, 'p2c_marker', 10)
        self.p3_c_publisher = self.create_publisher(Marker, 'p3c_marker', 10)

        self.transform_sub_ = self.create_subscription(
            VoiceCommand,
            '/transform',
            self.transform_callback,
            10)

        self.marker_list = []
        for i in range(7):
            temp_marker = Marker(header = Header(
                frame_id = "base_link"), id = 0, type = 2, action = 0, 
                scale = Vector3(x = 0.04, y = 0.04, z = 0.04), 
                color = ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0))
            self.marker_list.append(temp_marker)
        for i in range(3):
            self.marker_list[i+4].color = ColorRGBA(r = 1.0, g = 1.0, b = 1.0, a = 1.0)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # box dimensions
        self.length_box = 1.2
        self.width_box = 0.1
        self.height_box = 0.1
        self.la = self.length_box/2
        self.ha = self.height_box/2
        self.wa = self.width_box/2

        self.finger_r = 0.09

        # INITIAL AND FINAL CONFIGURATIONS
        # position of cm and contact points in space
        self.p_cm_init = np.array([0.7, 0, 0.9])
        self.p_cm_final = np.array([0.7, 0, 0.9])

        # Max Rotations: np.pi/16, np.pi/8, np.pi/16
        self.rot_init = np.array([0, 0, 0])
        self.rot_final = np.array([0, 0, 0])
        self.p_1 = np.array([0, self.la/2, self.ha])
        self.p_2 = np.array([0, 0, -self.ha])
        self.p_3 = np.array([0, -self.la/2, self.ha])
        self.p_1_c = np.array([0, self.la/2, self.ha + self.finger_r])
        self.p_2_c = np.array([0, 0, -(self.ha + self.finger_r)])
        self.p_3_c = np.array([0, -self.la/2, self.ha + self.finger_r])

        # TRANSLATIONS AND ROTATIONS
        self.dis = self.p_cm_final - self.p_cm_init
        self.rot_diff = self.rot_final - self.rot_init

        self.i = 0.0
        self.frames = 100
        self.prisma_msg = Prism()

    def transform_callback(self, msg):
ros2 topic pub --once /transform dexterous_msgs/msg/VoiceCommand "{c_opt: 0}"
ros2 topic pub --once /transform dexterous_msgs/msg/VoiceCommand "{c_opt: 1}"
ros2 topic pub --once /transform dexterous_msgs/msg/VoiceCommand "{c_opt: 2, transform: {linear: {x: 0, y: 0, z: 0.1}, angular: {x: 0.3, y: 0, z: 0}}}"



        if msg.c_opt == 0:
            self.finger_r = 0.07
            self.p_1_c = np.array([0, self.la/2, self.ha + self.finger_r])
            self.p_2_c = np.array([0, 0, -(self.ha + self.finger_r)])
            self.p_3_c = np.array([0, -self.la/2, self.ha + self.finger_r])
        elif msg.c_opt == 1:
            self.finger_r = 0.1
            self.p_1_c = np.array([0, self.la/2, self.ha + self.finger_r])
            self.p_2_c = np.array([0, 0, -(self.ha + self.finger_r)])
            self.p_3_c = np.array([0, -self.la/2, self.ha + self.finger_r])
        elif msg.c_opt == 2:
            if(self.i >= (self.frames + 1)):
                self.get_logger().error('a')
                self.dis = np.array([msg.transform.linear.x, msg.transform.linear.y, msg.transform.linear.z])
                self.rot_diff = np.array([msg.transform.angular.x, msg.transform.angular.y, msg.transform.angular.z])
                self.p_cm_init = self.p_cm_final
                self.rot_init = self.rot_final
                self.p_cm_final = self.p_cm_init + np.array([msg.transform.linear.x, msg.transform.linear.y, msg.transform.linear.z])
                self.rot_final = self.rot_init + np.array([msg.transform.angular.x, msg.transform.angular.y, msg.transform.angular.z])
                self.i = 0.0
            else:
                self.get_logger().error('b')

    def timer_callback(self):
        div = (self.i/self.frames)
        rot = self.rot_init + self.rot_diff * div
        trans = self.p_cm_init + self.dis * div
        
        # OBTAIN ROTATION MATRIX
        Rx = np.array([[1, 0, 0], [0, np.cos(rot[0]), -np.sin(rot[0])], [0, np.sin(rot[0]), np.cos(rot[0])]])
        Ry = np.array([[np.cos(rot[1]), 0, np.sin(rot[1])], [0, 1, 0], [-np.sin(rot[1]), 0, np.cos(rot[1])]])
        Rz = np.array([[np.cos(rot[2]), -np.sin(rot[2]), 0], [np.sin(rot[2]), np.cos(rot[2]), 0], [0, 0, 1]])
        R = Rx @ Ry @ Rz
                
        # PERFORM TRANSLATIONS
        p_cm_d = trans
        p1_d = np.dot(R, self.p_1) + trans
        p2_d = np.dot(R, self.p_2) + trans
        p3_d = np.dot(R, self.p_3) + trans
        p1_c_d = np.dot(R, self.p_1_c) + trans
        p2_c_d = np.dot(R, self.p_2_c) + trans
        p3_c_d = np.dot(R, self.p_3_c) + trans
        p_d = [p_cm_d, p1_d, p2_d, p3_d, p1_c_d, p2_c_d, p3_c_d]

        for j in range(7):
            p_temp = Point(x = p_d[j][0], y = p_d[j][1], z = p_d[j][2])
            self.prisma_msg.poses[j] = p_temp
            self.marker_list[j].pose.position = p_temp

        if self.i < (self.frames + 1):
            self.i += 1.0
        # else:
            # self.p_cm_init = self.p_cm_final
            # self.rot_init = self.rot_final

        self.prisma_publisher.publish(self.prisma_msg)
        self.p_cm_publisher.publish(self.marker_list[0])
        self.p1_publisher.publish(self.marker_list[1])
        self.p2_publisher.publish(self.marker_list[2])
        self.p3_publisher.publish(self.marker_list[3])
        self.p1_c_publisher.publish(self.marker_list[4])
        self.p2_c_publisher.publish(self.marker_list[5])
        self.p3_c_publisher.publish(self.marker_list[6])

def main(args=None):
    rclpy.init(args=args)
    m_p = Route()
    rclpy.spin(m_p)
    m_p.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
