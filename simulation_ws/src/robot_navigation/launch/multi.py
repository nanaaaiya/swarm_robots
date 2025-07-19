#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_rviz = LaunchConfiguration('enable_rviz')


    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('enable_rviz', default_value='true', description='Enable RViz')
    ])

    # Directories
    pkg_rb = get_package_share_directory('robot_bringup')
    pkg_nav = get_package_share_directory('robot_navigation')
    world_file = os.path.join(pkg_rb, 'worlds', 'swarm_world.worlds')
    urdf_file = os.path.join(pkg_rb, 'description', 'swarm_bot.urdf')
    map_yaml = os.path.join(pkg_nav, 'maps', 'map.yaml')
    params_file = os.path.join(pkg_nav, 'params', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'multi_robot_nav2.rviz')

    # Launch Gazebo
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        )
    ))

    # TF remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Map server
    ld.add_action(Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
        remappings=remappings
    ))
    ld.add_action(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]
    ))

    # Robot definitions
    robots = []
    for i in range(3):
        robots.append({
            'name': f'robot{i}',
            'x': 2.0 * i,
            'y': 0.0,
            'z': 0.01
        })

    # Sequence for each robot
    for robot in robots:
        ns = robot['name']

        # Spawn robot
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_rb, 'launch', 'robot_spawn_tf.py')
            ),
            launch_arguments={
                'robot_urdf': urdf_file,
                'robot_name': ns,
                'robot_namespace': ns,
                'x': TextSubstitution(text=str(robot['x'])),
                'y': TextSubstitution(text=str(robot['y'])),
                'z': TextSubstitution(text=str(robot['z'])),
                'use_sim_time': use_sim_time
            }.items()
        ))

        # Nav2 bringup after 5s
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                'namespace': ns,
                'use_namespace': 'True',
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'True',
                'slam': 'False'
            }.items()
        )
        ld.add_action(TimerAction(period=5.0, actions=[nav2_launch]))

        # Static TF publisher after 6s
        static_tf = Node(
            package='tf2_ros', executable='static_transform_publisher', name=f'static_map_to_odom_{ns}',
            namespace=ns,
            arguments=['0','0','0','0','0','0', 'map', 'odom'], output='screen'
        )
        ld.add_action(TimerAction(period=6.0, actions=[static_tf]))

        # RViz per robot after 8s
        rviz_node = Node(
            package='rviz2', executable='rviz2', name=f'{ns}_rviz', namespace=ns,
            arguments=['-d', rviz_config], output='screen',
            condition=IfCondition(enable_rviz)
        )
        ld.add_action(TimerAction(period=8.0, actions=[rviz_node]))

    return ld

