import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_name = get_package_share_directory('robot_bringup')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_name, 'launch', 'multi_robots_setup.py'),
        )
    )     

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(
                pkg_name, 'worlds', 'swarm_world.worlds'), ''],
            description='SDF world file'),
        gazebo,
        spawn_robot_world
    ])