import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        remappings=[
            ('/image_raw', 'people_video_frames'),
        ]
    )

    return LaunchDescription([
        camera_node,
    ])
