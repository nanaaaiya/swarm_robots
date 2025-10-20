#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import PushRosNamespace


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


    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            pkg_nav, 'rviz', 'multi_robot_nav2.rviz'),
        description='Full path to the RVIZ config file to use')

    ld.add_action(declare_rviz_config_file_cmd)

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

    
    # Robot definitions
    robots = []
    number_of_robots = 3
    for i in range(number_of_robots):
        robots.append({
            'name': f'robot{i}',
            'x': 2.0 * i,
            'y': 0.0,
            'z': 0.01
        })

    # Sequence for each robot
    for robot in robots:
        ns = robot['name']


        ld.add_action(Node(
            package='robot_state_publisher',
            namespace=ns,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0,
                            'frame_prefix': ns + '/'}],
            # remappings=remappings,
            arguments=[urdf_file],
        ))

        ld.add_action(Node(
        package='tf2_ros',
        namespace=ns,
        executable='static_transform_publisher',
        name=f'static_odom_to_base_link_{ns}',
        arguments=[
          '0','0','0','0','0','0',
          ns + '/odom', ns + '/base_link'
        ]
        ))

        ld.add_action(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace=ns,
            name='map_to_odom_broadcaster',
            arguments=[
                '0', '0', '0',    # x y z
                '0', '0', '0',    # roll pitch yaw
                'map', 'odom'     # parent frame, child frame
            ],
            # since namespace=ns, this publishes on:
            #   /robotX/tf_static
            #   /robotX/tf
        ))



            # Map server
        ld.add_action(Node(
            package='nav2_map_server', 
            executable='map_server', 
            name='map_server',
            namespace=ns,  
            output='screen', 
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': use_sim_time}],
            remappings=[('map', '/map')] 
        ))

        ld.add_action(Node(
            package='nav2_lifecycle_manager', 
            executable='lifecycle_manager', 
            name='lifecycle_manager_map_server',
            output='screen',
            namespace=ns,  
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'autostart': True}, 
                        {'node_names': ['map_server']}]
        ))



        # Spawn robot
        ld.add_action(Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            namespace=ns,
            name='spawn_' + ns,
            arguments=[
                '-entity', ns,
                '-file', urdf_file,
                '-robot_namespace', ns,
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z'])
            ],
            output='screen'
        ))

        # Nav2 bringup after 5s
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
            ),
            launch_arguments={
                # 'namespace': ns,
                # 'use_namespace': 'True',
                'map': map_yaml,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'True',
                'slam': 'False'
            }.items()
        )
        # ld.add_action(TimerAction(period=5.0, actions=[nav2_launch]))
        ld.add_action(
            TimerAction(
            period=5.0,
            actions=[
                GroupAction([
                PushRosNamespace(ns),
                nav2_launch
                ])
            ]
            )
        )


        # Static TF publisher after 6s
        # static_tf = Node(
        #     package='tf2_ros', executable='static_transform_publisher', name=f'static_odom_to_base_link_{ns}',
        #     namespace=ns,
        #     arguments=['0','0','0','0','0','0',  f'{ns}/odom', f'{ns}/base_link'], output='screen'
        # )
        # ld.add_action(TimerAction(period=6.0, actions=[static_tf]))

        # RViz per robot after 8s
        # rviz_node = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_nav, 'launch', 'rviz_launch.py')),
        #         launch_arguments={'use_sim_time': use_sim_time, 
        #                           'namespace': ns,
        #                           'use_namespace': 'True',
        #                           'rviz_config_file': rviz_config_file, 'log_level': 'warn'}.items(),
        #                            condition=IfCondition(enable_rviz)
        #                             )
        # ld.add_action(TimerAction(period=8.0, actions=[rviz_node]))

        # rviz = IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(pkg_nav, 'launch', 'rviz_launch.py')
        #     ),
        #     launch_arguments={
        #             'namespace':       ns,
        #             'use_namespace':  'True',
        #             'rviz_config_file': rviz_config_file,
        #             'log_level':      'warn'
        #         }.items()
        #         )

        # ld.add_action(
        # TimerAction(
        #     period=6.0,
        #     actions=[
        #     GroupAction([
        #         PushRosNamespace(ns),
        #         rviz
        #     ])
        #     ]
        # )
        # )

        

    return ld

