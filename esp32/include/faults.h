// Fault reporting (published on /system/faults by ros_iface).          OWNER: Razvi
// Codes and severities come from raspberry/src/pingpong_interfaces/msg/Fault.msg: one list for Pi and ESP32.
#pragma once
#include <stdint.h>
#include <pingpong_interfaces/msg/fault.h>

namespace fault {
constexpr uint16_t ESTOP = pingpong_interfaces__msg__Fault__ESTOP;
constexpr uint16_t HEARTBEAT_LOST = pingpong_interfaces__msg__Fault__HEARTBEAT_LOST;
constexpr uint16_t HOMING_TIMEOUT = pingpong_interfaces__msg__Fault__HOMING_TIMEOUT;
constexpr uint16_t LAUNCHER_JAM = pingpong_interfaces__msg__Fault__LAUNCHER_JAM;
constexpr uint16_t FIRE_REFUSED = pingpong_interfaces__msg__Fault__FIRE_REFUSED;
constexpr uint16_t AT_LIMIT = pingpong_interfaces__msg__Fault__AT_LIMIT;

constexpr uint8_t WARNING = pingpong_interfaces__msg__Fault__WARNING;
constexpr uint8_t ERROR = pingpong_interfaces__msg__Fault__ERROR;
constexpr uint8_t FATAL = pingpong_interfaces__msg__Fault__FATAL;
}  // namespace fault

struct FaultEvent {
  uint16_t code;
  uint8_t severity;
  bool active;
  char text[48];
};

// Queue a fault for publishing. Safe to call from anywhere in loop(), also while disconnected
// (the queue keeps the last 16 until the Pi is back).
void fault_report(uint16_t code, uint8_t severity, bool active, const char *text);
bool fault_pop(FaultEvent &out);
