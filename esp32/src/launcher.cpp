#include "launcher.h"

#include <Arduino.h>
#include <math.h>
#include <pingpong_interfaces/msg/launcher_command.h>
#include <pingpong_interfaces/msg/launcher_state.h>

#include "config.h"
#include "faults.h"
#include "safety.h"
#include "sensors.h"

namespace {

// states and commands: same numbers as the messages
constexpr uint8_t EMPTY = pingpong_interfaces__msg__LauncherState__EMPTY;
constexpr uint8_t LOADING = pingpong_interfaces__msg__LauncherState__LOADING;  // at hold, ball dropping
constexpr uint8_t COCKING = pingpong_interfaces__msg__LauncherState__COCKING;
constexpr uint8_t READY = pingpong_interfaces__msg__LauncherState__READY;
constexpr uint8_t FIRING = pingpong_interfaces__msg__LauncherState__FIRING;
constexpr uint8_t FIRED = pingpong_interfaces__msg__LauncherState__FIRED;
constexpr uint8_t JAM = pingpong_interfaces__msg__LauncherState__JAM;
// LOADED is not used: feeding is part of cocking

constexpr uint8_t CMD_LOAD = pingpong_interfaces__msg__LauncherCommand__LOAD;
constexpr uint8_t CMD_COCK = pingpong_interfaces__msg__LauncherCommand__COCK;
constexpr uint8_t CMD_FIRE = pingpong_interfaces__msg__LauncherCommand__FIRE;
constexpr uint8_t CMD_UNLOAD = pingpong_interfaces__msg__LauncherCommand__UNLOAD;
constexpr uint8_t CMD_RESET = pingpong_interfaces__msg__LauncherCommand__RESET;

constexpr uint8_t MOTOR_A_CHANNEL = 5;  // LEDC channels; FastAccelStepper does not use LEDC
constexpr uint8_t MOTOR_B_CHANNEL = 6;

// what the motor is doing inside COCKING / FIRING / EMPTY
enum class Move { NONE, HOMING, PARK, COCK, RECOCK, FIRE, UNLOAD };

uint8_t state = EMPTY;
uint32_t state_since_ms = 0;
Move move = Move::NONE;
uint32_t move_since_ms = 0;

bool homed = false;          // gear angle known (one release seen since power-up)
bool homing_allowed = true;  // auto-homing once after power-up, again after a RESET
int32_t zero_counts = 0;     // encoder counts at gear angle 0
float target_deg = 0.0f;     // unwrapped angle where the current move stops
bool released = false;       // FIRING: release angle passed
float homing_baseline = 0.0f;  // slow average of the gear speed while homing
uint32_t homing_sample_ms = 0;
uint32_t still_since_ms = 0;

// The holder must be EMPTY at power-up (gear homing releases the spring once).
// RESET = the operator filled it. The firmware simulation starts full for convenience.
int balls_left = SIM_HARDWARE ? MAGAZINE_CAPACITY : 0;
bool ball_loaded = false;
uint32_t shots_fired = 0;
uint32_t last_seq = 0;
int16_t motor_cmd = 0;
bool motor_brake = false;
bool changed = true;

// ------------------------------------------------------------ gear angle
float angle() { return (sensor_cock_counts() - zero_counts) * 360.0f / COCK_COUNTS_PER_REV; }  // unwrapped
float angle_in_turn() { return fmodf(fmodf(angle(), 360.0f) + 360.0f, 360.0f); }
bool at_hold() {
  float a = angle_in_turn();
  return a >= COCK_HOLD_DEG - COCK_HOLD_TOL_DEG && a < COCK_RELEASE_DEG;
}
// next unwrapped angle (ahead of now) that is `deg` within a turn
float next_angle(float deg) {
  float u = angle();
  return u + fmodf(deg - angle_in_turn() + 360.0f, 360.0f);
}

// ------------------------------------------------------------ DRV8871
void set_motor(int16_t cmd) {
  motor_cmd = constrain(cmd, -255, 255);
  motor_brake = false;
  ledcWrite(MOTOR_A_CHANNEL, motor_cmd > 0 ? motor_cmd : 0);
  ledcWrite(MOTOR_B_CHANNEL, motor_cmd < 0 ? -motor_cmd : 0);
}

void brake() {  // IN1 = IN2 = HIGH: windings shorted, holds against the spring
  motor_cmd = 0;
  motor_brake = true;
  ledcWrite(MOTOR_A_CHANNEL, 255);
  ledcWrite(MOTOR_B_CHANNEL, 255);
}

void go(uint8_t s, uint32_t now_ms) {
  state = s;
  state_since_ms = now_ms;
  changed = true;
}

void start(Move m, int16_t cmd, uint32_t now_ms) {
  move = m;
  move_since_ms = now_ms;
  still_since_ms = now_ms;
  set_motor(cmd);
}

void stop_move() {
  move = Move::NONE;
  set_motor(0);
}

void jam(const char *why, uint32_t now_ms) {
  stop_move();
  go(JAM, now_ms);
  fault_report(fault::LAUNCHER_JAM, fault::ERROR, true, why);
}

void start_homing(uint32_t now_ms) {  // turns slowly through one release to find it
  homing_allowed = false;
  homing_baseline = 0.0f;
  homing_sample_ms = now_ms;
  start(Move::HOMING, COCK_SLOW_DUTY, now_ms);
}

void start_cocking(uint32_t now_ms) {
  go(COCKING, now_ms);
  if (at_hold()) {  // already there (e.g. after RESET): just feed
    brake();
    move = Move::NONE;
    ball_loaded = balls_left > 0;
    go(LOADING, now_ms);
    return;
  }
  target_deg = next_angle(COCK_HOLD_DEG);
  start(Move::COCK, COCK_DUTY, now_ms);
}

// true while the motor is driven but the encoder does not move
bool stalled(uint32_t now_ms) {
  if (fabsf(sensor_cock_speed()) > 0.02f * COCK_COUNTS_PER_REV) still_since_ms = now_ms;
  return now_ms - still_since_ms > STALL_MS;
}

void update_homing(uint32_t now_ms) {
  float v = sensor_cock_speed();
  uint32_t t = now_ms - move_since_ms;
  if (t < 500) {  // let the motor spin up
    homing_baseline = v;
    return;
  }
  if (homing_baseline > 0.0f && v > RELEASE_SPEED_JUMP * homing_baseline) {
    // the spring just let go: we are a little past the release angle
    // TODO(Razvi): check RELEASE_DETECT_LAG_S on the real gear (hold must stay below 180 deg)
    float lag_counts = v * RELEASE_DETECT_LAG_S;
    zero_counts = sensor_cock_counts() - (int32_t)(COCK_RELEASE_DEG * COCK_COUNTS_PER_REV / 360.0f + lag_counts);
    homed = true;
    changed = true;
    target_deg = next_angle(COCK_FIRED_DEG);  // park in the bare half, spring relaxed
    start(Move::PARK, COCK_SLOW_DUTY, now_ms);
    return;
  }
  if (now_ms - homing_sample_ms >= 20) {  // one new speed sample every 20 ms
    homing_sample_ms = now_ms;
    homing_baseline += 0.2f * (v - homing_baseline);  // ~100 ms time constant: lags the jump
  }
  if (t > GEAR_HOMING_TIMEOUT_MS) jam("gear homing: no release seen", now_ms);
}

void update_cock(uint32_t now_ms) {
  float u = angle();
  if (u >= target_deg) {
    if (move == Move::PARK) {
      stop_move();
      return;
    }
    brake();
    move = Move::NONE;
    if (state == COCKING) {  // the plunger passed the holder opening: one ball dropped
      ball_loaded = balls_left > 0;
      if (ball_loaded) balls_left--;
      go(LOADING, now_ms);
    }
    return;
  }
  if (u >= target_deg - (COCK_HOLD_DEG - COCK_SLOW_DEG)) set_motor(COCK_SLOW_DUTY);
  if (stalled(now_ms) || now_ms - move_since_ms > COCK_TIMEOUT_MS) jam("cocking: hold angle not reached", now_ms);
}

void update_fire(uint32_t now_ms) {
  float u = angle();
  if (!released && u >= target_deg - (COCK_FIRED_DEG - COCK_RELEASE_DEG)) {
    released = true;
    if (ball_loaded) shots_fired++;
    ball_loaded = false;
    changed = true;
  }
  if (u >= target_deg) {
    stop_move();
    go(FIRED, now_ms);
    return;
  }
  if (!released && now_ms - move_since_ms > FIRE_RELEASE_TIMEOUT_MS) jam("spring not released", now_ms);
}

void update_unload(uint32_t now_ms) {
  float a = angle_in_turn();
  if (a < 2.0f || a > COCK_RELEASE_DEG) {  // rack back on its stop, spring relaxed
    stop_move();
    go(EMPTY, now_ms);
    if (ball_loaded) fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "UNLOAD: take the ball out by hand");
    ball_loaded = false;
    return;
  }
  if (now_ms - move_since_ms > UNLOAD_TIMEOUT_MS) jam("unload: spring not relaxed", now_ms);
}

}  // namespace

void launcher_setup() {
  ledcSetup(MOTOR_A_CHANNEL, 20000, 8);
  ledcAttachPin(PIN_COCK_MOTOR_A, MOTOR_A_CHANNEL);
  ledcSetup(MOTOR_B_CHANNEL, 20000, 8);
  ledcAttachPin(PIN_COCK_MOTOR_B, MOTOR_B_CHANNEL);
  set_motor(0);
}

void launcher_command(uint8_t cmd, uint32_t seq, uint32_t now_ms) {
  if (seq == last_seq) return;  // repeated message: a FIRE is never executed twice
  last_seq = seq;
  changed = true;  // publish the new last_cmd_seq = acknowledge

  if ((cmd == CMD_LOAD || cmd == CMD_COCK) && (state == EMPTY || state == FIRED)) {
    // doc P4: load = cock; the ball drops in on the way to the hold
    if (balls_left <= 0) {
      fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "LOAD refused: magazine empty");
    } else if (safety_estop()) {
      fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "LOAD refused: estop");
    } else if (!homed || move != Move::NONE) {
      fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "LOAD refused: gear not homed");
    } else {
      start_cocking(now_ms);
    }
  } else if (cmd == CMD_FIRE) {
    // ESP32 side of the fire permission (doc section 10, conditions marked ESP32)
    const char *refused = nullptr;
    if (state != READY) refused = "FIRE refused: not READY";
    else if (!ball_loaded) refused = "FIRE refused: no ball (magazine)";
    else if (!safety_arm_switch()) refused = "FIRE refused: arm switch OFF";
    else if (safety_estop()) refused = "FIRE refused: estop";
    else if (!safety_heartbeat_ok()) refused = "FIRE refused: no heartbeat";
    if (refused) {
      fault_report(fault::FIRE_REFUSED, fault::WARNING, true, refused);
    } else {
      released = false;
      target_deg = next_angle(COCK_FIRED_DEG);
      go(FIRING, now_ms);
      start(Move::FIRE, COCK_DUTY, now_ms);  // last tooth leaves the rack, the spring fires
    }
  } else if (cmd == CMD_UNLOAD) {
    // doc P12: let the spring down slowly by turning back. A fed ball stays in the barrel.
    if (state == READY && !safety_estop()) {
      go(COCKING, now_ms);
      start(Move::UNLOAD, -COCK_UNLOAD_DUTY, now_ms);
    } else {
      fault_report(fault::FIRE_REFUSED, fault::WARNING, true, "UNLOAD refused: not READY");
    }
  } else if (cmd == CMD_RESET) {
    if (state == JAM) {
      stop_move();
      fault_report(fault::LAUNCHER_JAM, fault::ERROR, false, "jam cleared");
      if (homed && at_hold()) {
        brake();
        go(READY, now_ms);
      } else {
        go(EMPTY, now_ms);  // TODO(Razvi): check this on the real mechanism
      }
    }
    if (state == EMPTY && !homed) homing_allowed = true;  // try the gear homing again
    balls_left = MAGAZINE_CAPACITY;  // operator refilled the holder
  }
}

void launcher_update(uint32_t now_ms) {
  if (safety_estop() && move != Move::NONE) {  // power is cut anyway; keep the output off too
    jam("estop during a launcher move", now_ms);
    return;
  }
  // Gear homing, once after power-up: needs the arm switch ON because it releases the
  // spring once. The holder is empty at power-up, so nothing leaves the barrel.
  if (!homed && homing_allowed && state == EMPTY && move == Move::NONE && safety_arm_switch() &&
      safety_heartbeat_ok() && !safety_estop()) {
    start_homing(now_ms);
  }

  switch (move) {
    case Move::HOMING: update_homing(now_ms); break;
    case Move::PARK:
    case Move::COCK:
    case Move::RECOCK: update_cock(now_ms); break;
    case Move::FIRE: update_fire(now_ms); break;
    case Move::UNLOAD: update_unload(now_ms); break;
    case Move::NONE: break;
  }

  if (state == LOADING && now_ms - state_since_ms > FEED_WAIT_MS) go(READY, now_ms);
  if (state == READY && move == Move::NONE && angle_in_turn() < COCK_HOLD_DEG - COCK_HOLD_TOL_DEG &&
      !safety_estop()) {
    // the spring pushed the gear back (spur gearbox under brake creeps): pull it up again
    target_deg = next_angle(COCK_HOLD_DEG);
    start(Move::RECOCK, COCK_SLOW_DUTY, now_ms);
  }
}

LauncherInfo launcher_info() {
  bool cocked = state == LOADING || state == READY;
  return LauncherInfo{state, ball_loaded, cocked, balls_left <= 0, safety_arm_switch(),
                      shots_fired, last_seq};
}

bool launcher_changed() {
  static LauncherInfo last{};
  LauncherInfo now = launcher_info();
  bool c = changed || now.ball_present != last.ball_present || now.cocked != last.cocked ||
           now.magazine_empty != last.magazine_empty || now.arm_switch != last.arm_switch;
  last = now;
  changed = false;
  return c;
}

int16_t launcher_motor_cmd() { return motor_cmd; }
bool launcher_motor_brake() { return motor_brake; }
