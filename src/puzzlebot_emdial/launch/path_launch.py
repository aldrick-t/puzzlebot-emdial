from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('puzzlebot_emdial')

        # Load the parameters from the YAML file
    config = os.path.join(
        package_directory,
        'config',
        'path_params.yaml'
    )

    x_path_generator = Node(
            package='puzzlebot_emdial',
            executable='x_path_generator',
            name='x_path_generator',
            parameters=[{'use_sim_time': False}, config],
        )

    x_path_control = Node(
            package='puzzlebot_emdial',
            executable='x_path_control',
            name='x_path_control',
            parameters=[{'use_sim_time': False}, config]
        )
    
    x_odometry_node = Node(
            package='puzzlebot_emdial',
            executable='x_odometry_node',
            name='x_odometry_node',
            parameters=[{'use_sim_time': False}]
        )

    return LaunchDescription([
        x_path_control,
        x_path_generator,
        x_odometry_node,
    ])