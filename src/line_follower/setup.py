from setuptools import find_packages, setup
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atad',
    maintainer_email='taldrick@gmail.com',
    description='A package for line following robot, built for ROS2 on a MCR2 Puzzlebot.',
    long_description='A package for line following robot, built for ROS2 on a MCR2 Puzzlebot. This package includes various nodes for camera preprocessing, line recognition, robot control, and traffic light recognition.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cam_preprocess = line_follower.cam_preprocess:main',
            'line_recogni = line_follower.line_recogni:main',
            'line_cmd = line_follower.line_cmd:main',
            'robot_ctrl = line_follower.robot_ctrl:main',
            'visual_monitor = line_follower.visual_monitor:main',
            'odometry_node = line_follower.odometry_node:main',
            'trafficlight_recogni = line_follower.trafficlight_recogni:main',
            'trafficlight_recogni_legacy = line_follower.trafficlight_recogni_legacy:main',
        ],
    },
)
