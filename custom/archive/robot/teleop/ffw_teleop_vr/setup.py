from setuptools import find_packages, setup
from glob import glob

package_name = 'ffw_teleop_vr'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)
setup(
    name=package_name,
    version='1.1.14',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/ffw_teleop_vr']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/cert', glob('cert/*.pem')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='FFW teleop ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_raw_sg2_consumer = ffw_teleop.vr_raw_sg2_consumer:main',
            'vr_raw_sg2_publisher = ffw_teleop.vr_raw_sg2_publisher:main',
            'vr_topic_monitor = ffw_teleop.vr_topic_monitor:main',
            'bimanual_ik_solver = ffw_teleop.pyroki_ik.bimanual_ik_solver:main',
        ],
    },
)
