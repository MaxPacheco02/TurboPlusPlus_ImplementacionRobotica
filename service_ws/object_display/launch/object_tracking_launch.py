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
    
    rviz_config = os.path.join(get_package_share_directory("object_display"),"rviz/","rviz.rviz")

    config = os.path.join(
        get_package_share_directory('object_display'),
        'config/',
        'params.yaml'
        )

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
        parameters=[config]
    )

    alarm_node = Node(
        package='object_display',
        executable='sound_node.py',
        name='sound_node'
    )


    # obj_pos_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('object_display'),
    #             'launch',
    #             'obj_pos_launch.py'
    #         ])
    #     ]),
    # )


    return LaunchDescription([
        alarm_node,
        # obj_pos_node,
        object_tracking_node,
        rviz_node
    ])

