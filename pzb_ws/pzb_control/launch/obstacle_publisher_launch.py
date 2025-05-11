import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns_', default_value='pzb')

    obstacle_publisher = Node(
        package='pzb_control',
        executable='obstacle_publisher',
        namespace=LaunchConfiguration('ns_'),
    )

    return LaunchDescription([
        ns_arg,

        obstacle_publisher,
    ])
