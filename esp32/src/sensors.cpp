#include "sensors.h"

#include <Arduino.h>

#include "config.h"
#include "launcher.h"

#if SIM_HARDWARE

// Simple physics of the launcher, timings close to doc section 11.
static bool ball = false, cocked = false;
static int balls_in_magazine = SIM_MAGAZINE_BALLS;
static uint32_t feeder_ms = 0, motor_ms = 0, release_ms = 0, last_ms = 0;

void sensors_setup() {}

void sensors_update(uint32_t now_ms) {
  uint32_t dt = now_ms - last_ms;
  last_ms = now_ms;

  feeder_ms = launcher_feeder_open() ? feeder_ms + dt : 0;
  if (feeder_ms > 80 && !ball && balls_in_magazine > 0) {
    ball = true;
    balls_in_magazine--;
  }

  motor_ms = launcher_motor_on() ? motor_ms + dt : 0;
  if (!cocked && motor_ms > 1000) {  // cocking takes 1 s
    cocked = true;
    motor_ms = 0;
  } else if (cocked && motor_ms > 30) {  // motor turning while cocked = release
    cocked = false;
    release_ms = now_ms;
  }
  if (release_ms && now_ms - release_ms > 40) {  // ball leaves 40 ms after release
    ball = false;
    release_ms = 0;
  }
}

bool sensor_ball() { return ball; }
bool sensor_cocked() { return cocked; }
bool sensor_range(float &meters) {
  meters = SIM_RANGE_M;
  return true;
}

#else

void sensors_setup() {
  pinMode(PIN_BALL_SENSOR, INPUT_PULLUP);
  pinMode(PIN_COCKED_SENSOR, INPUT_PULLUP);
  // TODO(Razvi): start the range sensor (UART2 / I2C) once the radar is chosen
}

void sensors_update(uint32_t) {
  // TODO(Razvi): read the range sensor here, validity check, median filter (doc S2)
}

// TODO(Razvi): debounce, the ball sensor can flicker when the ball rolls in
bool sensor_ball() { return digitalRead(PIN_BALL_SENSOR) == LOW; }
bool sensor_cocked() { return digitalRead(PIN_COCKED_SENSOR) == LOW; }
bool sensor_range(float &meters) {
  meters = 0.0f;
  return false;  // TODO(Razvi)
}

#endif
