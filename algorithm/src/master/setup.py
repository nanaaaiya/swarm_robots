from setuptools import setup
import os
from glob import glob 

package_name = 'master'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irs',
    maintainer_email='long.nguyen.210085@student.fulbright.edu.vn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aco = master.ACO:main",
            "statusReceiver = master.statusReceiver:main"
        ],
    },
)
