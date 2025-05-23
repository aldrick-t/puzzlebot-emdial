from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('mcHT_trafficlight')

        # Load the parameters from the YAML file
    config = os.path.join(
        package_directory,
        'config',
        'path_params.yaml'
    )
    
    configcv = os.path.join(
        package_directory,
        'config',
        'cv_params.yaml'
    )

    path_generator = Node(
            package='mcHT_trafficlight',
            executable='path_generator',
            name='path_generator',
            parameters=[{'use_sim_time': False}, config],
        )

    path_control = Node(
            package='mcHT_trafficlight',
            executable='path_control',
            name='path_control',
            parameters=[{'use_sim_time': False}, config]
        )
    
    odometry_node = Node(
            package='mcHT_trafficlight',
            executable='odometry_node',
            name='odometry_node',
            parameters=[{'use_sim_time': False}]
        )
    
    cv_decision_making = Node(
            package='mcHT_trafficlight',
            executable='cv_decision_making',
            name='cv_decision_making',
            parameters=[{'use_sim_time': False}, configcv]
        )
    
    plotter_monitor = Node(
            package='mcHT_trafficlight',
            executable='plotter_monitor',
            name='plotter_monitor',
            parameters=[{'use_sim_time': False}]
        )
    


    return LaunchDescription([
        cv_decision_making,
        path_generator,
        odometry_node,
        path_control,
        plotter_monitor,
    ])