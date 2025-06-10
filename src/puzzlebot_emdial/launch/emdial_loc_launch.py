'''
emdial_loc_launch.py

Launch file for LOCAL puzzlebot_emdial nodes in ONBOARD mode.
Only visual monitor and traffic light recognition nodes are launched.
Nodes: visual_monitor, trafficlight_recogni
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

    config_tlts = os.path.join(
        package_directory,
        'config',
        'tlts_params.yaml'
    )
    
    visual_monitor = Node(
            package='puzzlebot_emdial',
            executable='visual_monitor',
            name='visual_monitor',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',}]
        )
    
    tlts_detector = Node(
            package='puzzlebot_emdial',
            executable='tlts_detector',
            name='tlts_detector',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',}, config_cv, config_tlts]
        )
    
    return LaunchDescription([
        # visual_monitor,
        #trafficlight_recogni,
        tlts_detector,
    ])