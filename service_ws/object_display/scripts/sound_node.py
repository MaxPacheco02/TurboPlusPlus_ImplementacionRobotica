#!/usr/bin/env python3

import os

import rclpy
from playsound import playsound

from rclpy.node import Node
from std_msgs.msg import Int32 

from ament_index_python.packages import get_package_share_directory


class SoundNode(Node):

    def __init__(self):
        super().__init__('SoundNode')

        self.subscription = self.create_subscription(Int32, '/alarm', self.alarm_callback, 10)


        self.get_logger().info('Sound Node')


    def alarm_callback(self,msg):
        share_dir = get_package_share_directory('object_display')
        alarm_path = os.path.join(share_dir, 'resources', 'alarm.mp3')
        if(msg.data == 1):
            print(alarm_path)
            playsound(alarm_path)


def main(args=None):
    rclpy.init(args=args)
    node = SoundNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

