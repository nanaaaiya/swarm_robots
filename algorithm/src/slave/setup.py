from setuptools import setup
import os
from glob import glob  # Ensure glob is imported correctly

package_name = 'slave'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Use glob.glob
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),   # Use glob.glob
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irs',
    maintainer_email='long.nguyen.210085@student.fulbright.edu.vn',
    description='ROS2 package for slave robot nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation_process = slave.navigation_process:main",
            "odom = slave.odometry:main",
            "send_vel = slave.cmd_vel:main",
            "waypoint_navigator = slave.waypoint_navigator:main",
        ],
    },
)