#include "launcher.h"

#include <Arduino.h>
#include <pingpong_interfaces/msg/launcher_command.h>
#include <pingpong_interfaces/msg/launcher_state.h>

#include "config.h"
#include "faults.h"
#include "safety.h"
#include "sensors.h"

namespace {

// states and commands: same numbers as the messages
constexpr uint8_t EMPTY = pingpong_interfaces__msg__LauncherState__EMPTY;
constexpr uint8_t LOADING = pingpong_interfaces__msg__LauncherState__LOADING;
constexpr uint8_t LOADED = pingpong_interfaces__msg__LauncherState__LOADED;
constexpr uint8_t COCKING = pingpong_interfaces__msg__LauncherState__COCKING;
constexpr uint8_t READY = pingpong_interfaces__msg__LauncherState__READY;
constexpr uint8_t FIRING = pingpong_interfaces__msg__LauncherState__FIRING;
constexpr uint8_t FIRED = pingpong_interfaces__msg__LauncherState__FIRED;
constexpr uint8_t JAM = pingpong_interfaces__msg__LauncherState__JAM;

constexpr uint8_t CMD_LOAD = pingpong_interfaces__msg__LauncherCommand__LOAD;
constexpr uint8_t CMD_COCK = pingpong_interfaces__msg__LauncherCommand__COCK;
constexpr uint8_t CMD_FIRE = pingpong_interfaces__msg__LauncherCommand__FIRE;
constexpr uint8_t CMD_UNLOAD = pingpong_interfaces__msg__LauncherCommand__UNLOAD;
constexpr uint8_t CMD_RESET = pingpong_interfaces__msg__LauncherCommand__RESET;

constexpr uint8_t SERVO_CHANNEL = 4;  // LEDC channels; FastAccelStepper does not use LEDC
constexpr uint8_t MOTOR_CHANNEL = 5;

uint8_t state = EMPTY;
uint32_t state_since_ms = 0;
bool released = false;  // FIRING: spring has been released (cocked sensor went off)
bool dry_fire = false;  // FIRING without a ball (UNLOAD)
bool magazine_empty = false;
uint32_t shots_fired = 0;
uint32_t last_seq = 0;
bool feeder_open = false;
bool motor_on = false;
bool changed = true;

void set_feeder(bool open) {
  feeder_open = open;
  uint32_t us = open ? FEEDER_SERVO_OPEN_US : FEEDER_SERVO_CLOSED_US;
  ledcWrite(SERVO_CHANNEL, us * 65536UL / 20000UL);  // 50 Hz, 16 bit
}

void set_motor(bool on) {
  motor_on = on;
  ledcWrite(MOTOR_CHANNEL, on ? COCK_MOTOR_DUTY : 0);  // B stays LOW: one direction only
}

void go(uint8_t s, uint32_t now_ms) {
  state = s;
  state_since_ms = now_ms;
  changed = true;
}

void jam(const char *why, uint32_t now_ms) {
  set_motor(false);
  set_feeder(false);
  go(JAM, now_ms);
  fault_report(fault::LAUNCHER_JAM, fault::ERROR, true, why);
}

void start_feeding(uint32_t now_ms) {
  set_feeder(true);
  go(LOADING, now_ms);
}

void start_firing(bool dry, uint32_t now_ms) {
  released = false;
  dry_fire = dry;
  set_motor(true);  // sector gear turns past its last tooth and releases the plunger
  go(FIRING, now_ms);
}

}  // namespace

void launcher_setup() {
  ledcSetup(SERVO_CHANNEL, 50, 16);
  ledcAttachPin(PIN_FEEDER_SERVO, SERVO_CHANNEL);
  ledcSetup(MOTOR_CHANNEL, 20000, 8);
  ledcAttachPin(PIN_COCK_MOTOR_A, MOTOR_CHANNEL);
  pinMode(PIN_COCK_MOTOR_B, OUTPUT);
  digitalWrite(PIN_COCK_MOTOR_B, LOW);
  set_feeder(false);
  set_motor(false);
}

void launcher_command(uint8_t cmd, uint32_t seq, uint32_t now_ms) {
  if (seq == last_seq) return;  // repeated message: a FIRE is never executed twice
  last_seq = seq;
  changed = true;  // publish the new last_cmd_seq = acknowledge

  if (cmd == CMD_LOAD && (state == EMPTY || state == FIRED) && !magazine_empty) {
    start_feeding(now_ms);
  } else if (cmd == CMD_COCK && state == LOADED) {
    set_motor(true);
    go(COCKING, now_ms);
  } else if (cmd == CMD_FIRE) {
    // ESP32 side of the fire permission (doc section 10, conditions marked ESP32)
    const char *refused = nullptr;
    if (state != READY) refused = "FIRE refused: not READY";
    else if (!sensor_ball()) refused = "FIRE refused: no ball";
    else if (!safety_arm_switch()) refused = "FIRE refused: arm switch OFF";
    else if (safety_estop()) refused = "FIRE refused: estop";
    else if (!safety_heartbeat_ok()) refused = "FIRE refused: no heartbeat";
    if (refused) fault_report(fault::FIRE_REFUSED, fault::WARNING, true, refused);
    else start_firing(false, now_ms);
  } else if (cmd == CMD_UNLOAD) {
    // doc P12: release the spring safely = dry fire, ONLY when no ball is in the chamber.
    // TODO(Razvi): how does the operator remove the ball? (manual, or feeder backwards?)
    if (state == READY && !sensor_ball() && !safety_estop()) start_firing(true, now_ms);
    else fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "UNLOAD refused: remove ball first");
  } else if (cmd == CMD_RESET) {
    if (state == JAM) {
      set_motor(false);
      set_feeder(false);
      fault_report(fault::LAUNCHER_JAM, fault::ERROR, false, "jam cleared");
      go(sensor_cocked() ? READY : EMPTY, now_ms);  // TODO(Razvi): check this on the real mechanism
    }
    magazine_empty = false;  // operator refilled the magazine
  }
}

void launcher_update(uint32_t now_ms) {
  if (safety_estop() && motor_on) set_motor(false);  // power is cut anyway; keep the output off too
  uint32_t t = now_ms - state_since_ms;

  switch (state) {
    case LOADING:
      if (feeder_open && t > FEEDER_OPEN_MS) set_feeder(false);
      if (sensor_ball()) {
        set_feeder(false);
        go(LOADED, now_ms);
      } else if (t > BALL_CONFIRM_TIMEOUT_MS) {
        set_feeder(false);
        magazine_empty = true;
        go(EMPTY, now_ms);
      }
      break;
    case LOADED:  // doc P4: load = feed + cock
      if (!safety_estop()) {
        set_motor(true);
        go(COCKING, now_ms);
      }
      break;
    case COCKING:
      if (sensor_cocked()) {
        set_motor(false);
        go(READY, now_ms);
      } else if (safety_estop() || t > COCK_TIMEOUT_MS) {
        jam("cocking timeout", now_ms);
      }
      break;
    case FIRING:
      if (!released) {
        if (!sensor_cocked()) {
          released = true;
          set_motor(false);
          state_since_ms = now_ms;
        } else if (safety_estop() || t > FIRE_RELEASE_TIMEOUT_MS) {
          jam("spring not released", now_ms);
        }
      } else if (!sensor_ball()) {
        if (!dry_fire) shots_fired++;
        go(dry_fire ? EMPTY : FIRED, now_ms);
      } else if (t > BALL_EXIT_TIMEOUT_MS) {
        jam("ball did not leave", now_ms);
      }
      break;
    default:
      break;
  }
}

LauncherInfo launcher_info() {
  return LauncherInfo{state, sensor_ball(), sensor_cocked(), magazine_empty, safety_arm_switch(),
                      shots_fired, last_seq};
}

bool launcher_changed() {
  static LauncherInfo last{};
  LauncherInfo now = launcher_info();
  bool c = changed || now.ball_present != last.ball_present || now.cocked != last.cocked ||
           now.arm_switch != last.arm_switch;
  last = now;
  changed = false;
  return c;
}

bool launcher_feeder_open() { return feeder_open; }
bool launcher_motor_on() { return motor_on; }
