from setuptools import find_packages
from setuptools import setup

setup(
    name='communication_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('communication_msgs', 'communication_msgs.*')),
)
