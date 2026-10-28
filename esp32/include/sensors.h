// Sensors: cocking motor encoder = slip gear angle (doc stages S9, S10).        OWNER: Razvi
// There is no ball sensor and no cocked switch: the launcher works from the gear angle.
// The range sensor is on the Pi (pingpong_sensors/range_node).
// With SIM_HARDWARE the encoder is simulated from the cocking motor output.
#pragma once
#include <stdint.h>

void sensors_setup();
void sensors_update(uint32_t now_ms);
int32_t sensor_cock_counts();  // accumulated encoder counts, + = cocking direction
float sensor_cock_speed();     // [counts/s], updated every 20 ms
