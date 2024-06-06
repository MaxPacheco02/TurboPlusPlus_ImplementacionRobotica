import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_node = Node(
        package='pzb_vision',
        executable='camera_view',
    )

    line_detection_node = Node(
        package='pzb_vision',
        executable='line_detection_node2.py',
    )

    return LaunchDescription([
        # camera_node,
        line_detection_node,
    ])
