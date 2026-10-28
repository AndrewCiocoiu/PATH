# ESP32 firmware (micro-ROS)

The "reflexes" of the robot (doc section 2): motor motion, homing, launcher sequence,
sensors, watchdog and interlocks. Owners: **Razvi** (launcher, sensors, safety, micro-ROS),
**TT** (axes).

The ESP32 is a normal ROS2 node called `esp32`. It talks to the Pi over USB through the
**micro-ROS agent** and uses the **same messages** as the Pi (`../raspberry/src/pingpong_interfaces`).
The Pi cannot tell the difference between this board and `mock_hw_node`.
How micro-ROS works: section 2.4 of the [main README](../README.md). The Pi code is in
[`../raspberry/`](../raspberry/README.md).

## Build and flash

From the `ROS2/` folder, one level up (Docker/Podman does everything, no PlatformIO install needed):

```bash
./dev.sh fw build                          # first time ~10 min: builds micro-ROS + our messages
./dev.sh fw upload                         # ESP32 on /dev/ttyUSB0
PORT=/dev/ttyACM0 ./dev.sh fw upload       # other port
```

Or open this `esp32/` folder in VS Code with the PlatformIO extension.

## Test it with only an ESP32 on USB (no motors)

`include/config.h` has `SIM_HARDWARE 1` by default: axes and sensors are simulated on the
board. Flash it, then:

```bash
./dev.sh robot          # terminal 1: Pi software + micro-ROS agent, camera must exist
# or, just the link:
./dev.sh agent          # terminal 1
./dev.sh shell          # terminal 2
ros2 topic echo /turret/state
ros2 topic echo /launcher/state
```

LED: slow blink = waiting for the agent, on = connected, fast blink = e-stop.

## Files

| File | What | Owner |
|---|---|---|
| `include/config.h` | **all** pins, limits, timings, `SIM_HARDWARE` | Razvi, TT |
| `src/main.cpp` | `loop()`: real-time part + micro-ROS connection (reconnects by itself) | Razvi |
| `src/ros_iface.cpp` | the only file that uses micro-ROS: topics, callbacks, timestamps | Razvi |
| `src/safety.cpp` | e-stop, arm switch, 250 ms heartbeat watchdog | Razvi |
| `src/axes.cpp` | yaw/pitch: homing, soft limits, position/velocity modes (FastAccelStepper) | TT |
| `src/launcher.cpp` | feeder, cocking, FIRE with interlocks, jam detection | Razvi |
| `src/sensors.cpp` | ball, cocked, range sensor (+ simulation) | Razvi |
| `src/faults.cpp` | queue of faults for `/system/faults` | Razvi |

Find your work, from this folder: `grep -rn "TODO(Razvi)" src include` (or TT).

## Rules

1. **Safety never depends on the Pi.** Everything in `loop()` before `update_link()` runs
   whether the Pi is connected or not. If `/turret/cmd` stops for 250 ms, motion stops and
   FIRE is refused.
2. **Never block in `loop()`**: no `delay()`, no waiting for sensors. Use `millis()` and states,
   like `launcher.cpp` does.
3. **No `Serial.print()`**: the USB serial port is the micro-ROS link. Report problems as a
   fault (`fault_report(...)`), use the LED, or a USB-UART adapter on `Serial2`.
4. **Numbers come from the messages.** Fault codes, launcher states and turret modes are the
   generated constants (`pingpong_interfaces__msg__...`), never typed by hand.
5. **Changing a message** in `raspberry/src/pingpong_interfaces/msg/`: the next `./dev.sh fw build`
   notices it and rebuilds micro-ROS (a few minutes). Flash the ESP32 and rebuild the Pi
   together, or they won't understand each other.

## Limits to know

- Max 6 publishers and 4 subscribers (`colcon.meta`). Increase there if needed, then
  `./dev.sh fw rebuild-microros`.
- `ROS_SERIAL_BAUD` in `config.h` and the agent's `-b` (launch argument `baud`) must match.
