from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mcHT_trafficlight'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emanuelv',
    maintainer_email='A01710366@tec.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trafficlight = mcHT_trafficlight.trafficlight:main',
            'path_generator = mcHT_trafficlight.path_generator:main',
            'path_control = mcHT_trafficlight.path_control:main',
            'odometry_node = mcHT_trafficlight.odometry_node:main',
            'square_control = mcHT_trafficlight.square_control:main',
            'go_to_goal = mcHT_trafficlight.go_to_goal:main',
            'cv_decision_making = mcHT_trafficlight.cv_decision_making:main', 
            'webcam_publisher = scripts.webcam_publisher:main',
            'plotter_monitor = mcHT_trafficlight.plotter_monitor:main',
            'visual_monitor = mcHT_trafficlight.visual_monitor:main',
        ],
    },
)
