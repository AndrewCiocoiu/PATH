// Yaw and pitch axes: inner control loop, doc stage S9 and section 8.1.     OWNER: TT
// Step generation with acceleration limits runs in hardware (FastAccelStepper), so it keeps
// going smoothly even while micro-ROS is busy.
#pragma once
#include <stdint.h>

struct AxisState {
  float pos;    // rad
  float rate;   // rad/s
  bool homed;
  bool at_limit;
  bool moving;
};

void axes_setup();
void axes_update(uint32_t now_ms, bool motion_allowed, bool estop);
// mode: TurretCommand HOLD / POSITION / VELOCITY / HOME
void axes_command(uint8_t mode, float yaw, float pitch, float yaw_rate, float pitch_rate);
AxisState axes_yaw();
AxisState axes_pitch();
