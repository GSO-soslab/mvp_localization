import os
import yaml
import pathlib
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    node = Node(
        package='mvp_localization',
        executable='mvp_localization_node',
        name='mvp_localization',
        namespace="wamv_rise",
        output='screen',
        prefix=['stdbuf -o L'],
        remappings=[('imu/data', 'xsens_ahrs/imu/data'),
                        ('gps/fix', 'unicore_rtk/fix')],       
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        # parameters=[{
        #     'port': 8765,
        #     'address': '0.0.0.0',
        #     'send_buffer_limit': 10000000,
        #     'use_sim_time': False # Set to True if running in Gazebo
        # }]
    )

    return LaunchDescription([
        node,
        foxglove
    ])
