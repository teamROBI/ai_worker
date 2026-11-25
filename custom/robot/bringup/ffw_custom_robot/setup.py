from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ffw_custom_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/common'),
         glob('config/common/*')),
        (os.path.join('share', package_name, 'config/ffw_sg2l_sh5r_rev1_follower'),
         glob('config/ffw_sg2l_sh5r_rev1_follower/*')),
        (os.path.join('share', package_name,
                      'urdf/ffw_sg2l_sh5r_rev1_follower'),
         glob('urdf/ffw_sg2l_sh5r_rev1_follower/*')),
        (os.path.join('share', package_name,
                      'ros2_control/ffw_sg2l_sh5r_rev1_follower'),
         glob('ros2_control/ffw_sg2l_sh5r_rev1_follower/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Local mixed-gripper configuration for the AI Worker.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
