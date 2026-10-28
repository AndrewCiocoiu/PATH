// Sensors: ball-in-chamber, cocked, range (doc stages S2, S10).        OWNER: Razvi
// With SIM_HARDWARE they are simulated from the launcher's actuator outputs.
#pragma once
#include <stdint.h>

void sensors_setup();
void sensors_update(uint32_t now_ms);
bool sensor_ball();    // ball in the chamber
bool sensor_cocked();  // spring cocked
bool sensor_range(float &meters);  // returns false when there is no valid reading
