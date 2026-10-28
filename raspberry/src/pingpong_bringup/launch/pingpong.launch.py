"""Start Project PingPong.                       OWNER: TT

    ros2 launch pingpong_bringup pingpong.launch.py                 # simulation (default)
    ros2 launch pingpong_bringup pingpong.launch.py sim:=false      # real robot (ESP32 on USB)
    ros2 launch pingpong_bringup pingpong.launch.py sim:=false port:=/dev/ttyACM0
    ros2 launch pingpong_bringup pingpong.launch.py autonomy:=false # manual, use teleop_node
    ros2 launch pingpong_bringup pingpong.launch.py foxglove:=true  # live view in the browser

Arguments:
    sim        true: mock_hw_node + synthetic camera.   false: micro-ROS agent (ESP32) + real camera
    port, baud serial port of the ESP32 (sim:=false only), baud must match firmware config.h
    autonomy   false: no aim/controller/mission manager (for teleop and hardware tests)
    foxglove   true: start foxglove_bridge (ws://<pi-ip>:8765)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pingpong_bringup')
    params = os.path.join(share, 'config', 'pingpong.yaml')
    with open(os.path.join(share, 'urdf', 'pingpong.urdf')) as f:
        urdf = f.read()

    sim = LaunchConfiguration('sim')
    autonomy = LaunchConfiguration('autonomy')

    def node(package, executable, **kw):
        return Node(package=package, executable=executable, parameters=[params],
                    output='screen', emulate_tty=True, **kw)

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('autonomy', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='false'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='921600'),

        # S1, S3: vision (Sofian)
        Node(package='pingpong_vision', executable='camera_node', output='screen', emulate_tty=True,
             parameters=[params, {'source': PythonExpression(["'synthetic' if '", sim, "' == 'true' else 'opencv'"])}]),
        node('pingpong_vision', 'detector_node'),
        # S4: tracking (Sofian)
        node('pingpong_tracking', 'target_tracker_node'),
        # S5-S7: control (Andrei)
        node('pingpong_control', 'aim_solver_node', condition=IfCondition(autonomy)),
        node('pingpong_control', 'tracking_controller_node', condition=IfCondition(autonomy)),
        node('pingpong_control', 'mission_manager_node', condition=IfCondition(autonomy)),
        node('pingpong_control', 'safety_monitor_node', condition=IfCondition(autonomy)),
        # S8-S10: hardware (Razvi). Simulation: mock_hw_node. Real: micro-ROS agent <-> ESP32.
        node('pingpong_hw_bridge', 'mock_hw_node', condition=IfCondition(sim)),
        Node(package='micro_ros_agent', executable='micro_ros_agent', output='screen',
             arguments=['serial', '--dev', LaunchConfiguration('port'), '-b', LaunchConfiguration('baud')],
             condition=UnlessCondition(sim)),
        # frames (TT)
        node('pingpong_bringup', 'joint_state_relay'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        # S11: monitoring
        Node(package='foxglove_bridge', executable='foxglove_bridge',
             condition=IfCondition(LaunchConfiguration('foxglove'))),
    ])
