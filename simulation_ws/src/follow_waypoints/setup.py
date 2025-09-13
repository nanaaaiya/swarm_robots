from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'follow_waypoints'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='duong',
    maintainer_email='duong@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_waypoints_exe = follow_waypoints.follow_waypoints:main',
            'follow_waypoints_multi = follow_waypoints.follow_waypoints_multi:main'
        ],
    },
)
