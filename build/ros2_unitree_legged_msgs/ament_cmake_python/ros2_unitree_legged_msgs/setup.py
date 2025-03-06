from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2_unitree_legged_msgs',
    version='0.4.0',
    packages=find_packages(
        include=('ros2_unitree_legged_msgs', 'ros2_unitree_legged_msgs.*')),
)
