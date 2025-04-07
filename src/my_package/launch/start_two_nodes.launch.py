from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
            package='my_package',
            executable='my_publisher',
            output='screen')
    
    node2 = Node(
            package='my_package',
            executable='my_subscriber',
            output='screen')
    
    rqt_graph = Node(
            package='rqt_graph',
            executable='rqt_graph',
            output='screen')
    
    ld = LaunchDescription([node1, node2,rqt_graph])

    return ld