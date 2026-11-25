import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    custom_share = get_package_share_directory('ffw_custom_robot')
    bringup_launch = os.path.join(
        get_package_share_directory('ffw_bringup'), 'launch')
    serials_path = os.path.join(custom_share, 'config', 'common', 'rs_serial.yaml')
    with open(serials_path, 'r', encoding='utf-8') as serials_file:
        serials = yaml.safe_load(serials_file)

    camera_zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, 'camera_zed.launch.py')),
        launch_arguments={'camera_model': 'zedm'}.items(),
    )
    camera_realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, 'camera_realsense.launch.py')),
        launch_arguments={
            'serial_no1': serials['camera1_serial'],
            'serial_no2': serials['camera2_serial'],
        }.items(),
    )

    return LaunchDescription([
        camera_zed,
        TimerAction(period=10.0, actions=[camera_realsense]),
    ])
