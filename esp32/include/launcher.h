// Launcher: slip gear cocking motor with encoder, fire sequence (doc stage S9, sections 9.1 P4/P9, 11).
// OWNER: Razvi.  The ESP32 re-checks the FIRE interlocks itself (doc section 10).
// Mechanism: mechanics/BARREL/40 mm ball/parts. No feeder servo (the plunger is the feed gate)
// and no ball sensor: a ball drops by itself while cocking, the balls left are counted.
#pragma once
#include <stdint.h>

struct LauncherInfo {
  uint8_t state;  // LauncherState EMPTY .. JAM
  bool ball_present;    // estimated: a ball was fed during the last cock
  bool cocked;
  bool magazine_empty;  // estimated: balls counted since the last RESET (= refill)
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

// cocking motor output, read by the simulated encoder
int16_t launcher_motor_cmd();  // -255..255, + = cocking direction, 0 = coast
bool launcher_motor_brake();
