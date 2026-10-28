# Same ROS2 environment for everybody (laptops). The Pi 5 runs Ubuntu 24.04 + ROS2 Jazzy natively.
FROM docker.io/library/ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3-numpy python3-opencv python3-pytest \
        python3-colcon-common-extensions \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

# micro-ROS agent: the Pi end of the ESP32 link (doc section 7, option A).
# On the Pi without Docker run the same commands once (see README section 6).
RUN mkdir -p /opt/uros/src && cd /opt/uros/src \
    && git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git \
    && git clone --depth 1 -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git \
    && cd /opt/uros && . /opt/ros/jazzy/setup.sh \
    && colcon build --event-handlers console_direct- \
    && rm -rf build log src

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
