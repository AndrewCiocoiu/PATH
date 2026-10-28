import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pingpong_hw_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Razvi',
    maintainer_email='razvi@todo.todo',
    description='Mock ESP32 + world for simulation (replaces the ESP32 and the micro-ROS agent). Owner: Razvi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mock_hw_node = pingpong_hw_bridge.mock_hw_node:main',
        ],
    },
)
