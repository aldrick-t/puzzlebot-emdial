'''
line_follow_loc_launch.py

Launch file for LOCAL line_follower nodes in ONBOARD mode.
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
    package_directory = get_package_share_directory('line_follower')
    
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
    
    visual_monitor = Node(
            package='line_follower',
            executable='visual_monitor',
            name='visual_monitor',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',}]
        )
    
    traffic_light_detector = Node(
            package='line_follower',
            executable='traffic_light_detector',
            name='traffic_light_detector',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',}, config_cv]
        )
    
    # trafficlight_recogni = Node(
    #         package='line_follower',
    #         executable='trafficlight_recogni_mk2',
    #         name='trafficlight_recogni_mk2',
    #         parameters=[{'use_sim_time': False,
    #                      'camera_topic': 'video_source/raw',
    #                      }, config_cv]
    #     )
    
    return LaunchDescription([
        # visual_monitor,
        #trafficlight_recogni,
        traffic_light_detector,
    ])