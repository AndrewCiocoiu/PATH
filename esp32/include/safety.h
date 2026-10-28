// Safety: e-stop, arm switch, heartbeat watchdog (doc sections 2, 7, 10).   OWNER: Razvi
// Works WITHOUT the Pi: if /turret/cmd stops for HEARTBEAT_TIMEOUT_MS, motion stops and FIRE is blocked.
#pragma once
#include <stdint.h>

void safety_setup();
void safety_update(uint32_t now_ms);
void safety_heartbeat(uint32_t now_ms);  // call on every received /turret/cmd

bool safety_estop();          // emergency stop pressed
bool safety_arm_switch();     // hardware arm switch ON
bool safety_heartbeat_ok();
bool safety_motion_allowed(); // heartbeat ok and no e-stop
