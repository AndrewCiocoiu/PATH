import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pingpong_control'

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
    maintainer='Andrei',
    maintainer_email='andrei@todo.todo',
    description='Aim, motion control, mission manager, safety (stages S5-S7). Owner: Andrei.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aim_solver_node = pingpong_control.aim_solver_node:main',
            'tracking_controller_node = pingpong_control.tracking_controller_node:main',
            'mission_manager_node = pingpong_control.mission_manager_node:main',
            'safety_monitor_node = pingpong_control.safety_monitor_node:main',
        ],
    },
)
