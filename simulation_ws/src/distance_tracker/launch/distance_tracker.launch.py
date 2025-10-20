from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'robot_list': ['tb1', 'tb2', 'tb3'],
        'pose_topic_suffix': '/odom',
        'msg_type': 'odom',
        'use_per_robot_start_topic': True,
        'output_csv': '/home/hehe/Documents/GitHub/swarm_robots/simulation_ws/src/distance_tracker/robot_metrics.csv',
    }
    return LaunchDescription([
        Node(
            package='distance_tracker',
            executable='distance_tracker.py',  # file in scripts/
            name='distance_tracker',
            output='screen',
            parameters=[params],
        )
    ])
