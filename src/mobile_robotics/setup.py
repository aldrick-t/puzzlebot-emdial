from setuptools import find_packages, setup

package_name = 'mobile_robotics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atad',
    maintainer_email='atad@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = mobile_robotics.test_node:main',
            'move_fwd = mobile_robotics.move_fwd:main',
            'open_loop_ctrl = mobile_robotics.open_loop_ctrl:main',
            'move_fwd_dist = mobile_robotics.move_fwd_dist:main',
            'odometry_node = mobile_robotics.odometry_node:main',
            'go_to_goal = mobile_robotics.go_to_goal:main',
        ],
    },
)
