'''
line_follow_onb_launch.py

Launch file for ONBOARD line_follower nodes package in ONBOARD mode.
Nodes: line_recogni, cam_preprocess, line_cmd, robot_ctrl
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
    
    cam_preprocess = Node(
            package='line_follower',
            executable='cam_preprocess',
            name='cam_preprocess',
            parameters=[{'use_sim_time': False,
                         'camera_topic': 'video_source/raw',
                         }, config_cv]
        )
    
    line_recogni = Node(
            package='line_follower',
            executable='line_recogni',
            name='line_recogni',
            parameters=[{'use_sim_time': False,}, config_cv]
        )
    
    line_cmd = Node(
            package='line_follower',
            executable='line_cmd',
            name='line_cmd',
            parameters=[{'use_sim_time': False}]
        )
    
    robot_ctrl = Node(
            package='line_follower',
            executable='robot_ctrl',
            name='robot_ctrl',
            parameters=[{'use_sim_time': False}, config_ctrl]
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
        cam_preprocess,
        line_recogni,
        line_cmd,
        robot_ctrl,
        #trafficlight_recogni,
    ])