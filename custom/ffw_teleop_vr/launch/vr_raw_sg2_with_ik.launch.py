#!/usr/bin/env python3
#
# Scenario 2 (Relay) - Robot side: VR consumer + IK solver combined.
# Run this on the robot. Single launch for both the raw VR topic consumer
# and the PyRoKi bimanual IK solver.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'left_target_pose_topic',
            default_value='/l_goal_pose',
            description='Left arm target pose topic',
        ),
        DeclareLaunchArgument(
            'right_target_pose_topic',
            default_value='/r_goal_pose',
            description='Right arm target pose topic',
        ),
        DeclareLaunchArgument(
            'left_end_effector_link',
            default_value='arm_l_link7',
            description='Left arm end effector link name',
        ),
        DeclareLaunchArgument(
            'right_end_effector_link',
            default_value='arm_r_link7',
            description='Right arm end effector link name',
        ),
        DeclareLaunchArgument(
            'max_joint_step_rad',
            default_value='0.15',
            description='Max radians per cycle per joint before clamping',
        ),
        DeclareLaunchArgument(
            'max_joint_jump_rad',
            default_value='0.5',
            description='Reject IK solution if any joint jumps more than this',
        ),
        Node(
            package='ffw_teleop_vr',
            executable='vr_raw_sg2_consumer',
            name='vr_raw_sg2_consumer',
            output='screen',
        ),
        Node(
            package='ffw_teleop_vr',
            executable='bimanual_ik_solver',
            name='bimanual_ik_solver',
            output='screen',
            parameters=[{
                'left_target_pose_topic': LaunchConfiguration('left_target_pose_topic'),
                'right_target_pose_topic': LaunchConfiguration('right_target_pose_topic'),
                'left_end_effector_link': LaunchConfiguration('left_end_effector_link'),
                'right_end_effector_link': LaunchConfiguration('right_end_effector_link'),
                'max_joint_step_rad': LaunchConfiguration('max_joint_step_rad'),
                'max_joint_jump_rad': LaunchConfiguration('max_joint_jump_rad'),
            }],
        ),
    ])
