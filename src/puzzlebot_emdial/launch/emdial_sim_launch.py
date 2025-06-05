'''
emdial_sim_launch.py

Launch file for FULL puzzlebot_emdial package in SIMULATION mode.
Nodes: line_recogni, cam_preprocess, visual_monitor, line_cmd, robot_ctrl, trafficlight_recogni
Parameters: cv_params.yaml, control_params.yaml
'''

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('puzzlebot_emdial')
    
    # Load the parameters from the YAML file
    config_cv = os.path.join(
        package_directory,
        'config',
        'cv_params.yaml'
    )
    
    config_ctrl = os.path.join(
        package_directory,
        'config',
        'control_params.yaml'
    )
    
    line_recogni = Node(
            package='puzzlebot_emdial',
            executable='line_recogni',
            name='line_recogni',
            parameters=[{'use_sim_time': True,}, config_cv]
        )
    
    visual_monitor = Node(
            package='puzzlebot_emdial',
            executable='visual_monitor',
            name='visual_monitor',
            parameters=[{'use_sim_time': True,
                         'camera_topic': 'camera',}]
        )
    
    line_cmd = Node(
            package='puzzlebot_emdial',
            executable='line_cmd',
            name='line_cmd',
            parameters=[{'use_sim_time': True}]
        )
    
    robot_ctrl = Node(
            package='puzzlebot_emdial',
            executable='robot_ctrl',
            name='robot_ctrl',
            parameters=[{'use_sim_time': True,}, config_ctrl]
        )

    x_odometry_node = Node(
            package='puzzlebot_emdial',
            executable='x_odometry_node',
            name='x_odometry_node',
            parameters=[{'use_sim_time': True}]
    )
    
    tlts_detector = Node(
            package='puzzlebot_emdial',
            executable='tlts_detector',
            name='tlts_detector',
            parameters=[{'use_sim_time': True,
                         'camera_topic': 'camera',}, config_cv]
        )
    
    return LaunchDescription([
        line_recogni,
        visual_monitor,
        line_cmd,
        robot_ctrl,
        tlts_detector,
        x_odometry_node,
    ])
    