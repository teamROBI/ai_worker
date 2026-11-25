#!/usr/bin/env python3
#
# Scenario 2 (Relay) — Server side: Vuer server → raw ROS2 topic publisher.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('ffw_teleop_vr')
    default_cert = os.path.join(pkg_dir, 'cert', 'cert.pem')
    default_key = os.path.join(pkg_dir, 'cert', 'key.pem')

    return LaunchDescription([
        DeclareLaunchArgument('stream_fps', default_value='30'),
        DeclareLaunchArgument('vuer_port', default_value='8012'),
        DeclareLaunchArgument('cert_file', default_value=default_cert),
        DeclareLaunchArgument('key_file', default_value=default_key),
        Node(
            package='ffw_teleop_vr',
            executable='vr_raw_sg2_publisher',
            name='vr_raw_sg2_publisher',
            output='screen',
            parameters=[{
                'stream_fps': LaunchConfiguration('stream_fps'),
                'vuer_port': LaunchConfiguration('vuer_port'),
                'cert_file': LaunchConfiguration('cert_file'),
                'key_file': LaunchConfiguration('key_file'),
            }],
        ),
    ])
