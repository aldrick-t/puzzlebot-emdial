from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mc2_motioncontrol'

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
    maintainer='emdial',
    maintainer_email='taldrick@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_control = mc2_motioncontrol.path_control:main',
            'odometry_node = mc2_motioncontrol.odometry_node:main',
            'path_generator = mc2_motioncontrol.path_generator:main',
            'square_control = mc2_motioncontrol.square_control:main',
            'go_to_goal = mc2_motioncontrol.go_to_goal:main',
        ],
    },
)
