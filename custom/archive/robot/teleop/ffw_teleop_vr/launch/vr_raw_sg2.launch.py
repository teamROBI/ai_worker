#!/usr/bin/env python3
#
# Scenario 2 (Relay) - Robot side: raw VR topic consumer for SG2.
# Run this on the robot. Subscribes to /vr/raw/* topics published
# by vr_raw_sg2_publisher running on the server PC.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ffw_teleop_vr',
            executable='vr_raw_sg2_consumer',
            name='vr_raw_sg2_consumer',
            output='screen',
        ),
    ])
