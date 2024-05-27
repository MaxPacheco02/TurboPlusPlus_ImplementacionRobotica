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
    
    rviz_config = os.path.join(get_package_share_directory("pzb_control"),"rviz/","rviz.rviz")
    pzb_config = os.path.join(
        get_package_share_directory('pzb_control'),
        'config',
        'conf.yaml'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
    )

    state_node = Node(
        package='pzb_control',
        executable='state_estimate',
        name='state_estimate',
        parameters=[pzb_config],

        # Comment remappings for closed loop (REAL LIFE)
        # Include remappings for open loop (SIMULATIONS AND TESTING)
        remappings=[
            ('/VelocityEncL', '/VelocitySetL'),
            ('/VelocityEncR', '/VelocitySetR'),
        ],
    )

    guidance_node = Node(
        package='pzb_control',
        executable='guidance_node',
        name='guidance_node',
        parameters=[pzb_config],
    )

    master_node = Node(
        package='pzb_control',
        executable='master_node',
        name='master_node',
    )

    line_follower_path_publisher_node = Node(
        package='pzb_control',
        executable='line_follower_path_publisher_node',
        name='line_follower_path_publisher_node',
    )

    return LaunchDescription([
        state_node,
        guidance_node,
        rviz_node,
        line_follower_path_publisher_node,
        # master_node,
    ])
