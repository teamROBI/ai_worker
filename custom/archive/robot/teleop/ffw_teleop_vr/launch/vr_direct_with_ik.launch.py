#!/usr/bin/env python3
#
# Scenario 1 (Direct) — All nodes on robot.
# Quest 3 connects directly to the robot's WiFi AP.
# Runs: Vuer publisher + VR consumer + IK solver on the same machine.
#
# For Scenario 2 (Relay via server PC):
#   - Run vr_raw_sg2_publisher.launch.py on the server PC
#   - Run vr_raw_sg2_with_ik.launch.py on the robot

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
        # Publisher parameters
        DeclareLaunchArgument('stream_fps', default_value='30'),
        DeclareLaunchArgument('vuer_port', default_value='8012'),
        DeclareLaunchArgument('cert_file', default_value=default_cert),
        DeclareLaunchArgument('key_file', default_value=default_key),

        # Consumer parameters
        DeclareLaunchArgument('goal_pose_position_scale', default_value='1.1'),
        DeclareLaunchArgument('left_end_effector_link', default_value='arm_l_link7'),
        DeclareLaunchArgument('right_end_effector_link', default_value='arm_r_link7'),
        DeclareLaunchArgument('max_joint_step_rad', default_value='0.15'),
        DeclareLaunchArgument('max_joint_jump_rad', default_value='0.5'),

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
        Node(
            package='ffw_teleop_vr',
            executable='vr_raw_sg2_consumer',
            name='vr_raw_sg2_consumer',
            output='screen',
            parameters=[{
                'goal_pose_position_scale': LaunchConfiguration('goal_pose_position_scale'),
            }],
        ),
        Node(
            package='ffw_teleop_vr',
            executable='bimanual_ik_solver',
            name='bimanual_ik_solver',
            output='screen',
            parameters=[{
                'left_end_effector_link': LaunchConfiguration('left_end_effector_link'),
                'right_end_effector_link': LaunchConfiguration('right_end_effector_link'),
                'max_joint_step_rad': LaunchConfiguration('max_joint_step_rad'),
                'max_joint_jump_rad': LaunchConfiguration('max_joint_jump_rad'),
            }],
        ),
    ])
