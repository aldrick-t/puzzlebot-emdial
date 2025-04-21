from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    package_directory = get_package_share_directory('mc1_opencontrol')

        # Load the parameters from the YAML file
    config = os.path.join(
        package_directory,
        'config',
        'params_square.yaml'
    )

    square_path = Node(
            package='mc1_opencontrol',
            executable='square_path',
            name='square_path',
            parameters=[config]
        )

    return LaunchDescription([
        square_path
    ])

