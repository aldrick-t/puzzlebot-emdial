'''
line_follow_rem_launch.py

Launch file for FULL puzzlebot_emdial package in REMOTE mode.
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
    
    cam_preprocess = Node(
            package='puzzlebot_emdial',
            executable='cam_preprocess',
            name='cam_preprocess',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',
                         }, config_cv]
        )
    
    line_recogni = Node(
            package='puzzlebot_emdial',
            executable='line_recogni',
            name='line_recogni',
            parameters=[{'use_sim_time': False,}, config_cv]
        )
    
    visual_monitor = Node(
            package='puzzlebot_emdial',
            executable='visual_monitor',
            name='visual_monitor',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',}]
        )
    
    line_cmd = Node(
            package='puzzlebot_emdial',
            executable='line_cmd',
            name='line_cmd',
            parameters=[{'use_sim_time': False}]
        )
    
    robot_ctrl = Node(
            package='puzzlebot_emdial',
            executable='robot_ctrl',
            name='robot_ctrl',
            parameters=[{'use_sim_time': False}, config_ctrl]
        )
    
    trafficlight_recogni = Node(
            package='puzzlebot_emdial',
            executable='trafficlight_recogni_legacy',
            name='trafficlight_recogni_legacy',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',
                         }, config_cv]
        )
    
    return LaunchDescription([
        cam_preprocess,
        line_recogni,
        visual_monitor,
        line_cmd,
        robot_ctrl,
        trafficlight_recogni,
    ])