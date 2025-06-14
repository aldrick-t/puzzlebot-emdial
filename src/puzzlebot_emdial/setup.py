from setuptools import find_packages, setup
from glob import glob

package_name = 'puzzlebot_emdial'

setup(
    name=package_name,
    version='1.2.1',
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
    description='Puzzlebot EMDIAL package for ROS2',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_recogni = puzzlebot_emdial.line_recogni:main',
            'tlts_detector = puzzlebot_emdial.tlts_detector:main',
            'line_cmd = puzzlebot_emdial.line_cmd:main',
            'robot_ctrl = puzzlebot_emdial.robot_ctrl:main',
            'odometry_node = puzzlebot_emdial.odometry_node:main',
            'plotter_monitor = puzzlebot_emdial.plotter_monitor:main',
            'visual_monitor = puzzlebot_emdial.visual_monitor:main',
            'x_odometry_node = puzzlebot_emdial.x_odometry_node:main',
            'x_path_control = puzzlebot_emdial.x_path_control:main',
            'x_path_generator = puzzlebot_emdial.x_path_generator:main',
        ],
    },
)
