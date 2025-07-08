from launch import LaunchDescription
import launch.actions
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define robot parameters
    robot_urdf_path = launch.substitutions.LaunchConfiguration('robot_urdf')
    robot_name = launch.substitutions.LaunchConfiguration('robot_name')
    robot_namespace = launch.substitutions.LaunchConfiguration('robot_namespace')
    x = launch.substitutions.LaunchConfiguration('x')
    y = launch.substitutions.LaunchConfiguration('y')
    z = launch.substitutions.LaunchConfiguration('z')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )

    # Process the URDF file
    package_name = 'robot_bringup'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'description', 'swarm_bot.xarco')
    robot_description_config = xacro.process_file(xacro_file)

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    # Path to your URDF file
    # urdf_file_path = '/home/irs/swarm_robot/simulation_ws/src/robot_bringup/descriptions/robot_swarm.urdf'

    # # Load URDF content into a variable
    # with open(urdf_file_path, 'r') as urdf_file:
    #     robot_description_content = urdf_file.read()

    # Robot State Publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=robot_namespace,  # Unique namespace for each robot
        # parameters=[{'use_sim_time': True, 'robot_description': robot_description_content}]
        parameters=[params]
    )

    # Spawn Robot node
    spawn_robot_node = launch_ros.actions.Node(
        package='robot_bringup',
        executable='robot_spawn_position.py',
        output='screen',
        arguments=[
            '--robot_urdf', robot_urdf_path,
            '--robot_name', robot_name,
            '--robot_namespace', robot_namespace,
            '-x', x,
            '-y', y,
            '-z', z
        ]
    )

    # Joint State Publisher node
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        namespace=robot_namespace,  # Same namespace as robot
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        # Declare parameters
        launch.actions.DeclareLaunchArgument('robot_urdf', description='Path to robot URDF file'),
        launch.actions.DeclareLaunchArgument('robot_name', description='Name of the robot'),
        launch.actions.DeclareLaunchArgument('robot_namespace', description='Namespace of the robot'),
        launch.actions.DeclareLaunchArgument('x', default_value='0.0', description='Initial x position'),
        launch.actions.DeclareLaunchArgument('y', default_value='0.0', description='Initial y position'),
        launch.actions.DeclareLaunchArgument('z', default_value='0.0', description='Initial z position'),

        # Nodes
        robot_state_publisher_node,
        spawn_robot_node,
        joint_state_publisher_node  # Add the joint_state_publisher node here
    ])
