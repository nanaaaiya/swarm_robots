import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    # Names and poses of the robots
    number_of_robots = 3
    robots = []

    for i in range(number_of_robots):
        robot_name = "Robot"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})

    
    
    # # urdf = os.path.join(get_package_share_directory('robot_bringup'), 'descriptions/', 'robot_swarm.urdf')
    # pkg_robot_description = get_package_share_directory('robot_bringup')
    # # assert os.path.exists(urdf), "The robot.urdf doesnt exist in "+str(urdf)   
    
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    enable_drive = LaunchConfiguration('enable_drive', default='false')
    declare_enable_drive = DeclareLaunchArgument(
        name='enable_drive', default_value=enable_drive, description='Enable robot drive node'
    )

    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    declare_enable_rviz = DeclareLaunchArgument(
        name='enable_rviz', default_value=enable_rviz, description='Enable rviz launch'
    )

    
    # turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    # package_dir = get_package_share_directory('turtlebot3_multi_robot')
    # nav_launch_dir = os.path.join(package_dir, 'launch', 'nav2_bringup')

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    map_dir = get_package_share_directory('robot_navigation')


    rviz_config_file = LaunchConfiguration('rviz_config_file')
    
    package_name = 'robot_bringup'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    urdf = os.path.join(pkg_path, 'description/', 'swarm_bot.urdf')
    map = os.path.join(map_dir, 'map', 'map.yaml')

    world = os.path.join(
        get_package_share_directory('robot_bringup'),
        'worlds', 'swarm_world.worlds')
    
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

     
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_enable_drive)
    ld.add_action(declare_enable_rviz)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
 
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map,
                     },],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])


    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    ######################


    last_action = None
    # Spawn turtlebot3 instances in gazebo
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        # Create state publisher node for that instance
        start_state_publisher = Node(
            package='robot_state_publisher',
            namespace=namespace,
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                            'publish_frequency': 10.0}],
            remappings=remappings,
            arguments=[urdf],
        )

        # Create spawn call ->> Node or IncludeLaunchDescription?
        spawn = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(package_name, 'launch',
                                                           'robot_spawn_tf.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name'],
                                  'use_sim_time': use_sim_time
                }.items())

        bringup_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
                    launch_arguments={  
                                    'slam': 'False',
                                    'namespace': namespace,
                                    'use_namespace': 'True',
                                    'map': map,
                                    'map_server': 'False',
                                    'params_file': params_file,
                                    'default_bt_xml_filename': os.path.join(
                                        get_package_share_directory('nav2_bt_navigator'),
                                        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                                    'autostart': 'true',
                                    'use_sim_time': use_sim_time, 'log_level': 'warn'}.items()
                                    )

        if last_action is None:
            # Call add_action directly for the first robot to facilitate chain instantiation via RegisterEventHandler
            ld.add_action(start_state_publisher)
            ld.add_action(spawn)
            ld.add_action(bringup_cmd)

        else:
            # Use RegisterEventHandler to ensure next robot creation happens only after the previous one is completed.
            # Simply calling ld.add_action for spawn_entity introduces issues due to parallel run.
            spawn_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn,
                            start_state_publisher,
                            bringup_cmd],
                )
            )

            ld.add_action(spawn_event)

        # Save last instance for next RegisterEventHandler
        last_action = spawn
    ######################

    ######################
    # Start rviz nodes and drive nodes after the last robot is spawned
    for robot in robots:

        namespace = '/' + robot['name'] 

        

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'rviz_launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
                                   condition=IfCondition(enable_rviz)
                                    )
        # post_spawn_event = RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=last_action,
        #         on_exit=[initial_pose_cmd, rviz_cmd],
        #     )
        # )
        # last_action = initial_pose_cmd

        # ld.add_action(post_spawn_event)
        ld.add_action(rviz_cmd)




    ######################

    return ld