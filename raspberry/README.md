# Raspberry Pi code (ROS2)

Everything that runs on the Raspberry Pi 5: seeing, tracking, aiming and deciding (doc
stages S1 to S7). It is a normal ROS2 Jazzy **colcon workspace**: the packages are in `src/`,
and building creates `build/`, `install/` and `log/` next to it (never commit those).
Owners: **Sofian** (vision, tracking), **Andrei** (control, mission), **Razvi** (range sensor,
simulated ESP32, teleop), **TT** (messages, launch, URDF).

How the whole system works, who owns which file and the team rules are in the
[main README](../README.md). The ESP32 firmware is in [`../esp32/`](../esp32/README.md).

All commands below run from the `ROS2/` folder, one level up.

## Packages

| Package | What | Owner |
|---|---|---|
| `pingpong_interfaces` | all messages (doc section 6), shared with the ESP32 firmware | TT (team decision) |
| `pingpong_vision` | `camera_node` (S1), `detector_node` (S3) | Sofian |
| `pingpong_tracking` | `target_tracker_node` (S4) | Sofian |
| `pingpong_control` | `aim_solver_node` (S5), `mission_manager_node` (S6), `tracking_controller_node` (S7), `safety_monitor_node` | Andrei |
| `pingpong_sensors` | `range_node` (S2): VL53L1X range sensor on the Pi I2C bus, real robot only | Razvi |
| `pingpong_hw_bridge` | `mock_hw_node`: simulated ESP32, range sensor and target, for `./dev.sh sim` | Razvi |
| `pingpong_bringup` | launch file, `config/pingpong.yaml`, URDF, `teleop_node` | TT, Razvi |

```bash
./dev.sh build                                     # build everything
./dev.sh build --packages-select pingpong_vision   # only one package (faster)
./dev.sh test                                      # unit tests
grep -rn "TODO(Sofian)" raspberry/src              # your work (or Andrei, Razvi, TT)
```

## Playing with the simulation

While `./dev.sh sim` runs, from `./dev.sh shell`:

```bash
ros2 param set /mock_hw_node estop true                     # press the emergency stop -> ESTOP
ros2 param set /mock_hw_node estop false
ros2 topic pub --once /operator/reset std_msgs/msg/Empty    # operator reset -> HOMING
ros2 param set /mock_hw_node arm_switch false               # hardware arm switch off -> IDLE
ros2 param set /mock_hw_node target_motion_amplitude 0.4    # faster moving target
ros2 param set /aim_solver_node use_lead false              # see what lead does
ros2 bag record -a -x "/camera/image_raw"                   # record a test (doc section 14)
```

Manual mode (no autonomy, keyboard drives the turret):

```bash
./dev.sh sim autonomy:=false                             # terminal 1
./dev.sh run "ros2 run pingpong_bringup teleop_node"     # terminal 2
```

## On the real robot (Pi 5)

With `sim:=false` the launch file starts the **micro-ROS agent** instead of `mock_hw_node`
(the agent is the bridge to the ESP32, see section 2.4 of the main README), `range_node` for the
VL53L1X on the Pi I2C bus (`pip install vl53l1x`, enable I2C), and the camera in
`opencv` mode (still a TODO, it tells you at start-up).

Start-up with the real launcher (no ball sensor, see `../esp32/README.md`): power up with the
ball holder **empty**, switch the arm switch ON (the ESP32 homes the slip gear with one empty
release), fill the holder, then `ros2 topic pub --once /operator/reset std_msgs/msg/Empty`.

```bash
./dev.sh fw build && ./dev.sh fw upload      # flash the ESP32 first (see ../esp32/README.md)
./dev.sh robot                               # ESP32 on /dev/ttyUSB0
./dev.sh robot port:=/dev/ttyACM0            # other port
```

Setting up the Pi itself: Ubuntu 24.04 + ROS2 Jazzy, copy the `ROS2/` folder, and build the
micro-ROS agent once (the same commands as in `docker/Dockerfile`):

```bash
mkdir -p ~/uros/src && cd ~/uros/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
cd ~/uros && source /opt/ros/jazzy/setup.bash && colcon build
echo "source ~/uros/install/setup.bash" >> ~/.bashrc
```
