import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('pzb_vision'),
        'config/',
        'params.yaml'
        )

    color_detection = Node(
        package='pzb_vision',
        executable='color_detection',
        name='color_detection',
        parameters= [config]
    )

    return LaunchDescription([color_detection])
