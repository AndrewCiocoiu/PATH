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

#define PIN_COCK_MOTOR_A 18          // DRV8871 IN1 (PWM forward = cocking direction)
#define PIN_COCK_MOTOR_B 19          // DRV8871 IN2 (PWM reverse); IN1 = IN2 = HIGH brakes
#define PIN_COCK_ENC_A 16            // JGA25-370 Hall encoder, channel A (3.3 V logic)
#define PIN_COCK_ENC_B 17            // channel B

#define PIN_ESTOP_SENSE 35           // input only, external pull-up; HIGH = e-stop PRESSED (NC loop open)
#define PIN_ARM_SWITCH 39            // input only, external pull-up; LOW = armed

// No feeder servo and no ball / cocked sensors: the plunger is the feed gate and the
// cocking motor encoder gives the slip gear angle. The range sensor (VL53L1X) is on the Pi.

// ---------------------------------------------------------------- axes (doc section 8.3)
// Angles in radians. yaw positive = counter-clockwise from above, pitch positive = barrel up.
#define STEPS_PER_REV 3200.0f        // 17HS4401S-0.9: 400 full steps x 8 microsteps (TMC2209 MS1=MS2=GND)
#define YAW_GEAR_RATIO 3.0f          // motor turns per head turn, printed gears, PLACEHOLDER
#define PITCH_GEAR_RATIO 3.0f
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
// Slip gear angle in degrees, 0 = first tooth meets the rack, 180 = last tooth releases.
// See mechanics/BARREL/40 mm ball/parts (sheets 1, 3, 4).
#define COCK_ENC_PPR 11.0f           // encoder pulses per MOTOR turn (x4 quadrature in software)
#define COCK_GEARBOX_RATIO 90.0f     // motor turns per output turn, from the motor label, PLACEHOLDER
#define COCK_COUNTS_PER_REV (COCK_ENC_PPR * 4.0f * COCK_GEARBOX_RATIO)

#define COCK_CLEAR_DEG 150.0f        // plunger has cleared the holder opening, the ball drops
#define COCK_SLOW_DEG 145.0f         // slow down from here to stop precisely at the hold
#define COCK_HOLD_DEG 170.0f         // pre-cocked hold, 10 deg before release
#define COCK_HOLD_TOL_DEG 2.0f       // re-cock if the spring pushes the gear back further than this
#define COCK_RELEASE_DEG 180.0f      // last tooth leaves the rack
#define COCK_FIRED_DEG 195.0f        // stop here after a shot (bare half under the rack)

#define COCK_DUTY 255                // 0..255 PWM, normal cocking
#define COCK_SLOW_DUTY 90            // approach to the hold, re-cock, homing
#define COCK_UNLOAD_DUTY 70          // reverse, lets the spring down slowly

#define FEED_WAIT_MS 150             // ball drop into the barrel: 0.09 s + margin
#define MAGAZINE_CAPACITY 5          // balls in a full holder (no ball sensor: counted)

#define COCK_TIMEOUT_MS 3000         // hold angle not reached -> JAM
#define FIRE_RELEASE_TIMEOUT_MS 300  // release angle not reached -> JAM
#define UNLOAD_TIMEOUT_MS 3000
#define GEAR_HOMING_TIMEOUT_MS 6000  // more than one slow turn without a release -> JAM
#define RELEASE_SPEED_JUMP 1.6f      // homing: speed after / before > this = the spring let go
#define RELEASE_DETECT_LAG_S 0.03f   // the jump is seen ~1.5 speed samples late, measure on the gear
#define STALL_MS 150                 // no encoder movement while driving for this long = stalled

// ---------------------------------------------------------------- publish rates
#define TURRET_STATE_PERIOD_MS 20    // 50 Hz
#define LAUNCHER_STATE_PERIOD_MS 200 // 5 Hz + on every change

#if SIM_HARDWARE
#define SIM_COCK_RPM 60.0f           // simulated output speed at full duty, no load
#define SIM_GEAR_START_DEG 250.0f    // gear angle at power-up (unknown on the real robot)
#endif
