import sys
import os
import xacro

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution



def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "Robot"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})


    return robots 

def generate_launch_description():
    package_name = 'robot_bringup'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    urdf = os.path.join(pkg_path, 'description/', 'swarm_bot.xacro')
    
    # urdf = os.path.join(get_package_share_directory('robot_bringup'), 'descriptions/', 'robot_swarm.urdf')
    pkg_robot_description = get_package_share_directory('robot_bringup')
    # assert os.path.exists(urdf), "The robot.urdf doesnt exist in "+str(urdf)

    # Names and poses of the robots
    robots = gen_robot_list(3)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_robot_description, 'launch',
                                                           'robot_spawn_tf.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld