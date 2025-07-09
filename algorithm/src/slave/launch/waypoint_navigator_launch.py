# ~/ros2_ws/src/slave/launch/waypoint_navigator_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('slave')
    config_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    return LaunchDescription([
        Node(
            package='slave',
            executable='waypoint_navigator',
            name='waypoint_navigator_node',
            output='screen',
            parameters=[config_path]
        )
    ])