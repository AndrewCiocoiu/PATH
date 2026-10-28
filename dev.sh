#!/bin/bash
# Project PingPong helper. Run ./dev.sh without arguments for help.
# Laptops: runs everything inside the ROS2 container (Docker or Podman).
# Pi / native ROS2 install: runs directly.
#   raspberry/  Pi code, colcon workspace (build/ install/ log/ end up in there)
#   esp32/      ESP32 firmware, PlatformIO
set -e
cd "$(dirname "$0")"
IMAGE=localhost/pingpong-ros:jazzy
FW_IMAGE=localhost/pingpong-fw:latest
PORT=${PORT:-/dev/ttyUSB0}   # ESP32 serial port, e.g.  PORT=/dev/ttyACM0 ./dev.sh fw upload

if command -v ros2 >/dev/null 2>&1; then
    run() { cd raspberry && bash -c "source /opt/ros/\${ROS_DISTRO:-jazzy}/setup.bash; [ -f install/setup.bash ] && source install/setup.bash; $*"; }
    build_image() { echo "Native ROS2 found, no container needed."; }
fi

# container engine (also needed natively for the firmware, unless PlatformIO is installed)
if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
    ENGINE=docker; USER_FLAGS=(--user "$(id -u):$(id -g)"); VOL="$PWD:/ws"
elif command -v podman >/dev/null 2>&1; then
    ENGINE=podman; USER_FLAGS=(--userns=keep-id); VOL="$PWD:/ws:Z"
fi
TTY=(); [ -t 0 ] && TTY=(-it)
DEV=(); for d in /dev/ttyUSB0 /dev/ttyACM0 "$PORT"; do [ -e "$d" ] && DEV+=(--device "$d"); done

if ! command -v ros2 >/dev/null 2>&1; then
    [ -n "$ENGINE" ] || { echo "Install Docker (https://docs.docker.com/engine/install/) or Podman first."; exit 1; }
    run() {
        "$ENGINE" run --rm "${TTY[@]}" "${USER_FLAGS[@]}" "${DEV[@]}" \
            --network host --ipc host -v "$VOL" -w /ws/raspberry \
            -e HOME=/tmp -e ROS_LOG_DIR=/ws/raspberry/log/ros -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
            "$IMAGE" bash -c "[ -f install/setup.bash ] && source install/setup.bash; $*"
    }
    build_image() { "$ENGINE" build -t "$IMAGE" docker/; }
fi

# PlatformIO for the ESP32: native if installed, otherwise in its own container
pio_run() {
    if command -v pio >/dev/null 2>&1; then
        (cd esp32 && pio "$@")
    else
        [ -n "$ENGINE" ] || { echo "Install PlatformIO, Docker or Podman first."; exit 1; }
        "$ENGINE" image inspect "$FW_IMAGE" >/dev/null 2>&1 \
            || "$ENGINE" build -t "$FW_IMAGE" -f docker/Dockerfile.firmware docker/
        "$ENGINE" run --rm "${TTY[@]}" "${USER_FLAGS[@]}" "${DEV[@]}" -e HOME=/tmp \
            -e PLATFORMIO_CORE_DIR=/ws/esp32/.platformio_core -w /ws/esp32 -v "$VOL" "$FW_IMAGE" pio "$@"
    fi
}

case "$1" in
    setup) build_image ;;
    build) run "colcon build --symlink-install ${*:2}" ;;
    sim)   run "colcon build --symlink-install && ros2 launch pingpong_bringup pingpong.launch.py ${*:2}" ;;
    robot) run "ros2 launch pingpong_bringup pingpong.launch.py sim:=false ${*:2}" ;;
    test)  run "colcon build --symlink-install && colcon test ${*:2}; colcon test-result --verbose" ;;
    shell) run "bash" ;;
    run)   run "${*:2}" ;;
    agent) run "ros2 run micro_ros_agent micro_ros_agent serial --dev $PORT -b 921600" ;;
    fw)    case "$2" in
               build)   pio_run run ;;
               upload)  pio_run run -t upload --upload-port "$PORT" ;;
               rebuild-microros) pio_run run -t clean_microros && pio_run run ;;
               *) echo "Usage: ./dev.sh fw build | upload | rebuild-microros"; exit 1 ;;
           esac ;;
    clean) rm -rf raspberry/build raspberry/install raspberry/log ;;
    *) cat <<'HELP'
Usage: ./dev.sh <command>

  setup            first time only: build the ROS2 container image (~5 min)
  sim [args]       build + start everything in simulation (no hardware needed)
                   e.g.  ./dev.sh sim foxglove:=true
  shell            open a terminal inside the ROS2 environment (use a 2nd one next to sim)
  build [args]     colcon build   e.g. ./dev.sh build --packages-select pingpong_vision
  test             build + run all unit tests
  robot [args]     start on the real robot (Pi, sim:=false): micro-ROS agent + ESP32
                   e.g.  ./dev.sh robot port:=/dev/ttyACM0
  agent            only the micro-ROS agent, to test the ESP32 alone
  fw build         compile the ESP32 firmware (first time ~10 min: builds micro-ROS)
  fw upload        flash the ESP32 on $PORT (default /dev/ttyUSB0)
  fw rebuild-microros   after changing messages, if the automatic rebuild did not happen
  run "<cmd>"      run one command, e.g. ./dev.sh run "ros2 topic echo /system/state"
  clean            delete raspberry/build/ install/ log/
HELP
    ;;
esac
