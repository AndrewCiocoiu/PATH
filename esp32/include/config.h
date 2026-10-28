// Project PingPong ESP32 configuration: ALL pins, limits and timings live here.   OWNERS: Razvi, TT
// Values marked PLACEHOLDER must be replaced once the hardware is chosen (T05, T07).
#pragma once

// ---------------------------------------------------------------- build mode
// 1 = no motors or sensors attached: axes and sensors are simulated on the ESP32.
//     Lets you test the micro-ROS link and the Pi software with a bare ESP32 board on USB.
// 0 = real hardware.
#ifndef SIM_HARDWARE
#define SIM_HARDWARE 1
#endif

// ---------------------------------------------------------------- micro-ROS link
#define ROS_SERIAL_BAUD 921600       // must match the agent:  micro_ros_agent serial -b 921600
#define ROS_NODE_NAME "esp32"
#define AGENT_PING_PERIOD_MS 500     // how often we check that the Pi is still there

// ---------------------------------------------------------------- safety (doc sections 2, 7, 10)
#define HEARTBEAT_TIMEOUT_MS 250     // no /turret/cmd for this long -> stop motors, block FIRE

// ---------------------------------------------------------------- pins (ESP32 DevKitC, PLACEHOLDER)
#define PIN_LED 2

#define PIN_YAW_STEP 25
#define PIN_YAW_DIR 26
#define PIN_PITCH_STEP 27
#define PIN_PITCH_DIR 14
#define PIN_STEPPER_ENABLE 13        // shared enable, LOW = enabled (TMC2209)

#define PIN_YAW_HOME_SWITCH 32       // INPUT_PULLUP, pressed = LOW
#define PIN_PITCH_HOME_SWITCH 33

#define PIN_FEEDER_SERVO 23
#define PIN_COCK_MOTOR_A 18          // DC motor driver inputs (e.g. DRV8871)
#define PIN_COCK_MOTOR_B 19
#define PIN_COCKED_SENSOR 16         // INPUT_PULLUP, cocked = LOW
#define PIN_BALL_SENSOR 17           // INPUT_PULLUP, ball present = LOW

#define PIN_ESTOP_SENSE 35           // input only, external pull-up; HIGH = e-stop PRESSED (NC loop open)
#define PIN_ARM_SWITCH 39            // input only, external pull-up; LOW = armed

// Range sensor: TODO(Razvi) when the radar is chosen (UART2 or I2C on 21/22).

// ---------------------------------------------------------------- axes (doc section 8.3)
// Angles in radians. yaw positive = counter-clockwise from above, pitch positive = barrel up.
#define STEPS_PER_REV 3200.0f        // 200 full steps x 16 microsteps, PLACEHOLDER
#define YAW_GEAR_RATIO 4.0f          // motor turns per head turn, PLACEHOLDER
#define PITCH_GEAR_RATIO 4.0f
#define YAW_INVERT_DIR false
#define PITCH_INVERT_DIR false

#define YAW_MIN_RAD -2.8f            // soft limits (also in the URDF)
#define YAW_MAX_RAD 2.8f
#define PITCH_MIN_RAD -0.35f
#define PITCH_MAX_RAD 0.8f

#define YAW_MAX_RATE 2.0f            // rad/s
#define YAW_MAX_ACCEL 8.0f           // rad/s^2
#define PITCH_MAX_RATE 1.5f
#define PITCH_MAX_ACCEL 6.0f

#define HOMING_RATE 0.4f             // rad/s, slow
#define YAW_HOME_SWITCH_RAD -2.9f    // angle at which the home switch triggers, PLACEHOLDER
#define PITCH_HOME_SWITCH_RAD -0.4f
#define HOMING_TIMEOUT_MS 20000

// ---------------------------------------------------------------- launcher (doc section 11)
#define FEEDER_SERVO_CLOSED_US 1000
#define FEEDER_SERVO_OPEN_US 1700
#define FEEDER_OPEN_MS 150           // gate open time, one ball drops
#define BALL_CONFIRM_TIMEOUT_MS 500  // no ball after this -> magazine empty
#define COCK_TIMEOUT_MS 2500         // cocked sensor not reached -> JAM
#define FIRE_RELEASE_TIMEOUT_MS 300  // spring not released -> JAM
#define BALL_EXIT_TIMEOUT_MS 200     // ball still in chamber after release -> JAM
#define COCK_MOTOR_DUTY 255          // 0..255

// ---------------------------------------------------------------- publish rates
#define TURRET_STATE_PERIOD_MS 20    // 50 Hz
#define LAUNCHER_STATE_PERIOD_MS 200 // 5 Hz + on every change
#define RANGE_PERIOD_MS 66           // 15 Hz

#if SIM_HARDWARE
#define SIM_RANGE_M 2.5f             // simulated range sensor reading
#define SIM_MAGAZINE_BALLS 10
#endif
