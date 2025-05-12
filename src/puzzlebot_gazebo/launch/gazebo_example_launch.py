from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable, 
                            IncludeLaunchDescription, SetLaunchConfiguration)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


#IMPORTS REQUIRED TO SET THE PACKAGE ADDRESS (DIRECTORIES)
import os
from ament_index_python.packages import get_package_share_directory

#IMPORTS REQUIRED FOR Launching Nodes
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#IMPORTS REQUIRED FOR EVENTS AND ACTIONS
from launch.actions import  AppendEnvironmentVariable, EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import EnvironmentVariable, Command


def generate_launch_description():

    # World and robot file names
    #world_file = 'puzzlebot_arena.world' # Default MCR2 world
    world_file = 'puzzlebot_arena.world' # Default MCR2 world
    robot = 'puzzlebot_jetson_lidar_ed'

    # Robot's initial position
    pos_x = '0.0'   # X coordinate
    pos_y = '0.0'   # Y coordinate
    pos_th = '0.0'   # th angle

    # Simulation time and pause settings
    sim_time = 'true'  # Set to 'true' for sim time
    pause_gazebo = 'false' # Set to 'true' to start Gazebo in paused mode

    # Verbosity level for Gazebo logs (higher means more detailed logs)
    gazebo_verbosity = 4

    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    models = get_package_share_directory('puzzlebot_description')
    gazebo_resources = get_package_share_directory('puzzlebot_gazebo')

    # Paths for world and robot description
    robot_path = os.path.join(models, 'urdf', 'mcr2_robots',  f"{robot}.xacro")
    world_path = os.path.join(gazebo_resources, 'worlds', world_file)
    gazebo_models_path = os.path.join(gazebo_resources, 'models')
    gazebo_plugins_path = os.path.join(gazebo_resources,'plugins')
    gazebo_media_path = os.path.join(gazebo_models_path,'models', 'media', 'materials')
    default_ros_gz_bridge_config_file_path = os.path.join(gazebo_resources, 'config', f"{robot}.yaml")

    #  DECLARE LAUNCH ARGUMENTS  
    declare_x_arg = DeclareLaunchArgument('x', default_value=pos_x, description='X position of the robot')
    declare_y_arg = DeclareLaunchArgument('y', default_value=pos_y, description='Y position of the robot')
    declare_th_arg = DeclareLaunchArgument('yaw', default_value=pos_th, description='angle of the robot')
    declare_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=sim_time, description='Use simulated time')
    declare_pause_arg = DeclareLaunchArgument('pause', default_value=pause_gazebo, description='Start Gazebo paused')


    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    use_sim_time = LaunchConfiguration('use_sim_time')
    pause = LaunchConfiguration('pause')


    # Set Gazebo environment variables to include models and textures
    set_gazebo_resources = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{gazebo_models_path}:{gazebo_media_path}"
    )

    set_gazebo_plugins = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=f"{gazebo_plugins_path}"
    )

    
        # Robot description using xacro
    robot_description = Command(['xacro ', str(robot_path)])

    # Path to Gazebo launch script
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])


    start_gazebo_server = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args':  ['-r' f'-v {gazebo_verbosity} ', world_path],  #-r Run inmediately, -s Server Mode (headless) -v 4 Verbosity level 4 (more detailed logs on terminal)
                'on_exit_shutdown': 'true',
                'pause': pause
            }.items(),
        )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": use_sim_time,
        }],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "puzzlebot",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-Y", yaw,
        ],
        output="screen",
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge_cmd = Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    parameters=[{
                        'config_file': default_ros_gz_bridge_config_file_path,
                    }],
                    output='screen')
    
    start_gazebo_ros_image_bridge_cmd = None
    if robot != "puzzlebot_hacker_ed":
        start_gazebo_ros_image_bridge_cmd = Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['camera']
        )
    

 
    l_d = [
        declare_x_arg, declare_y_arg, declare_th_arg, declare_sim_time_arg, declare_pause_arg,
        set_gazebo_resources, set_gazebo_plugins, robot_state_publisher_node,start_gazebo_server, spawn_robot, start_gazebo_ros_bridge_cmd]

    # Conditionally add the image bridge
    if start_gazebo_ros_image_bridge_cmd:
        l_d.append(start_gazebo_ros_image_bridge_cmd)

    return LaunchDescription(l_d)









