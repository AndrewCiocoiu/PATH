#!/bin/bash
# Source ROS2, the micro-ROS agent and, if built, our workspace, then run the given command.
source /opt/ros/jazzy/setup.bash
source /opt/uros/install/setup.bash
[ -f /ws/raspberry/install/setup.bash ] && source /ws/raspberry/install/setup.bash
exec "$@"
