import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true'
    )
    
    # Process the URDF file
    package_name = 'robot_bringup'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'description', 'swarm_bot.xarco')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        # namespace="Robot0",
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        # remappings=[
        #     ("/scan", "laser_controller/out"),  # Remap LiDAR topic
        #     ("/odom", "odom"),  # Remap odom topic
        # ],
    )
    
    # Include Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')])
    )
    
    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # namespace="/Robot0",
        arguments=['-topic', 'robot_description', '-entity', 'swarm_bot'],
        output='screen',
    )
    
    # Return combined LaunchDescription
    return LaunchDescription([
        declare_use_sim_time,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
    ])
