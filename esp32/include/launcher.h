// Launcher: feeder gate, cocking motor, fire sequence (doc stage S9, sections 9.1 P4/P9, 11).
// OWNER: Razvi.  The ESP32 re-checks the FIRE interlocks itself (doc section 10).
#pragma once
#include <stdint.h>

struct LauncherInfo {
  uint8_t state;  // LauncherState EMPTY .. JAM
  bool ball_present;
  bool cocked;
  bool magazine_empty;
  bool arm_switch;
  uint32_t shots_fired;
  uint32_t last_cmd_seq;
};

void launcher_setup();
void launcher_update(uint32_t now_ms);
// cmd: LauncherCommand LOAD / COCK / FIRE / UNLOAD / RESET. A repeated seq is ignored.
void launcher_command(uint8_t cmd, uint32_t seq, uint32_t now_ms);
LauncherInfo launcher_info();
bool launcher_changed();  // true once after every change (for publish-on-change)

// actuator outputs, read by the simulated sensors
bool launcher_feeder_open();
bool launcher_motor_on();
