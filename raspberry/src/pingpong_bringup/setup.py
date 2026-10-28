import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'pingpong_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TT',
    maintainer_email='tt@todo.todo',
    description='Launch files, parameters, URDF, teleop. Owners: TT (URDF/launch), Razvi (teleop).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = pingpong_bringup.teleop_node:main',
            'joint_state_relay = pingpong_bringup.joint_state_relay:main',
        ],
    },
)
