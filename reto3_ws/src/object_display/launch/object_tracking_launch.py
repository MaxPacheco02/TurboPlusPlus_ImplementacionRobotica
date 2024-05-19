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
    
    rviz_config = os.path.join(get_package_share_directory("object_display"),"rviz/","rviz.rviz")

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    object_tracking_node = Node(
        package='object_display',
        executable='object_tracking',
        name='object_tracking',
    )


    return LaunchDescription([
        object_tracking_node,
        rviz_node
    ])

