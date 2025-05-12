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
        'cv_params.yaml'
    )
    
    cv_decision_making = Node(
            package='mcHT_trafficlight',
            executable='cv_decision_making',
            name='cv_decision_making',
            parameters=[{'use_sim_time': True}, config]
        )
    
    return LaunchDescription([
        cv_decision_making,
    ])