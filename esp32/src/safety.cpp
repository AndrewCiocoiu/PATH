#include "safety.h"

#include <Arduino.h>

#include "config.h"
#include "faults.h"

static uint32_t last_heartbeat_ms = 0;
static bool ever_heard = false;
static bool heartbeat_ok = false;
static bool estop = false;
static bool arm_switch = false;

void safety_setup() {
#if !SIM_HARDWARE
  pinMode(PIN_ESTOP_SENSE, INPUT);  // GPIO 34-39 have no internal pull-ups: external resistor
  pinMode(PIN_ARM_SWITCH, INPUT);
#endif
}

void safety_heartbeat(uint32_t now_ms) {
  last_heartbeat_ms = now_ms;
  ever_heard = true;
}

void safety_update(uint32_t now_ms) {
#if SIM_HARDWARE
  bool new_estop = false;
  arm_switch = true;
#else
  // TODO(Razvi): debounce both inputs (e.g. must be stable for 20 ms)
  bool new_estop = digitalRead(PIN_ESTOP_SENSE) == HIGH;
  arm_switch = digitalRead(PIN_ARM_SWITCH) == LOW;
#endif
  if (new_estop != estop) {
    estop = new_estop;
    fault_report(fault::ESTOP, fault::FATAL, estop, "emergency stop");
  }

  bool ok = ever_heard && (now_ms - last_heartbeat_ms) < HEARTBEAT_TIMEOUT_MS;
  if (ever_heard && ok != heartbeat_ok) {
    fault_report(fault::HEARTBEAT_LOST, fault::ERROR, !ok,
                 ok ? "heartbeat back" : "no TurretCommand for 250 ms");
  }
  heartbeat_ok = ok;
}

bool safety_estop() { return estop; }
bool safety_arm_switch() { return arm_switch; }
bool safety_heartbeat_ok() { return heartbeat_ok; }
bool safety_motion_allowed() { return heartbeat_ok && !estop; }
