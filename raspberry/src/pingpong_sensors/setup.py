from setuptools import find_packages, setup

package_name = 'pingpong_sensors'

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
    description='Sensors wired to the Raspberry Pi: VL53L1X range sensor (stage S2). Owner: Razvi.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'range_node = pingpong_sensors.range_node:main',
        ],
    },
)
