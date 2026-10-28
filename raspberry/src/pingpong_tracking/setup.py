import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pingpong_tracking'

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
    maintainer='Sofian',
    maintainer_email='sofian@todo.todo',
    description='Target tracking and fusion (stage S4). Owner: Sofian.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_tracker_node = pingpong_tracking.target_tracker_node:main',
        ],
    },
)
