#include "axes.h"

#include <Arduino.h>
#include <FastAccelStepper.h>
#include <math.h>
#include <pingpong_interfaces/msg/turret_command.h>

#include "config.h"
#include "faults.h"

namespace {

constexpr uint8_t HOLD = pingpong_interfaces__msg__TurretCommand__HOLD;
constexpr uint8_t POSITION = pingpong_interfaces__msg__TurretCommand__POSITION;
constexpr uint8_t VELOCITY = pingpong_interfaces__msg__TurretCommand__VELOCITY;
constexpr uint8_t HOME = pingpong_interfaces__msg__TurretCommand__HOME;

enum class Homing { NOT_HOMED, SEEK_SWITCH, PARK, DONE };

#if !SIM_HARDWARE
FastAccelStepperEngine engine;
#endif

class Axis {
 public:
  Axis(const char *name, float gear, float min, float max, float max_rate, float max_accel,
       float switch_pos)
      : name_(name), steps_per_rad_(STEPS_PER_REV * gear / (2.0f * PI)), min_(min), max_(max),
        max_rate_(max_rate), max_accel_(max_accel), switch_pos_(switch_pos) {}

  void begin(uint8_t step_pin, uint8_t dir_pin, bool invert, uint8_t switch_pin) {
    switch_pin_ = switch_pin;
#if SIM_HARDWARE
    sim_pos_ = 0.3f;  // unknown position at power-on, homing finds it
#else
    pinMode(switch_pin, INPUT_PULLUP);
    stepper_ = engine.stepperConnectToPin(step_pin);
    stepper_->setDirectionPin(dir_pin, !invert);
    stepper_->setEnablePin(PIN_STEPPER_ENABLE, true);  // low active
    stepper_->setAutoEnable(false);
    stepper_->enableOutputs();
    stepper_->setAcceleration(to_steps(max_accel_));
#endif
  }

  // called at 100 Hz from axes_update()
  void update(uint32_t now_ms, bool allowed, float dt) {
    if (!allowed) {
      drive_rate(0.0f, dt);
    } else if (mode_ == HOME) {
      update_homing(now_ms, dt);
    } else if (homing_ != Homing::DONE) {
      drive_rate(0.0f, dt);  // never move freely before homing: position is unknown
    } else if (mode_ == POSITION) {
      drive_position(constrain(target_pos_, min_, max_), dt);
    } else if (mode_ == VELOCITY) {
      float r = constrain(target_rate_, -max_rate_, max_rate_);
      if ((pos() >= max_ && r > 0) || (pos() <= min_ && r < 0)) r = 0.0f;  // soft limit
      drive_rate(r, dt);
    } else {
      drive_rate(0.0f, dt);  // HOLD
    }
    bool limit = homing_ == Homing::DONE && (pos() >= max_ - 1e-3f || pos() <= min_ + 1e-3f);
    if (limit != at_limit_) {
      at_limit_ = limit;
      fault_report(fault::AT_LIMIT, fault::WARNING, limit, name_);
    }
  }

  void command(uint8_t mode, float pos, float rate) {
    mode_ = mode;
    target_pos_ = pos;
    target_rate_ = rate;
  }

  void lost_position() { homing_ = Homing::NOT_HOMED; }  // e.g. after e-stop cut motor power

  AxisState state() const {
    return AxisState{pos(), rate(), homing_ == Homing::DONE, at_limit_, fabsf(rate()) > 1e-3f};
  }

 private:
  void update_homing(uint32_t now_ms, float dt) {
    switch (homing_) {
      case Homing::NOT_HOMED:  // first HOME command: start seeking the switch
        homing_ = Homing::SEEK_SWITCH;
        homing_started_ms_ = now_ms;
        break;
      case Homing::SEEK_SWITCH:
        if (switch_pressed()) {
          set_pos(switch_pos_);
          homing_ = Homing::PARK;
        } else if (now_ms - homing_started_ms_ > HOMING_TIMEOUT_MS) {
          drive_rate(0.0f, dt);
          homing_ = Homing::NOT_HOMED;
          mode_ = HOLD;
          homing_failed_ = true;
          fault_report(fault::HOMING_TIMEOUT, fault::ERROR, true, name_);
        } else {
          drive_rate(switch_pos_ < 0 ? -HOMING_RATE : HOMING_RATE, dt);
        }
        break;
      case Homing::PARK:  // park position = 0, 0 (barrel level, forward)
        drive_position(0.0f, dt);
        if (fabsf(pos()) < 0.002f && fabsf(rate()) < 1e-3f) {
          homing_ = Homing::DONE;
          if (homing_failed_) fault_report(fault::HOMING_TIMEOUT, fault::ERROR, false, name_);
          homing_failed_ = false;
        }
        break;
      case Homing::DONE:
        drive_rate(0.0f, dt);
        break;
    }
  }

  bool switch_pressed() {
#if SIM_HARDWARE
    return switch_pos_ < 0 ? sim_pos_ <= switch_pos_ : sim_pos_ >= switch_pos_;
#else
    return digitalRead(switch_pin_) == LOW;  // TODO(Razvi): debounce
#endif
  }

#if SIM_HARDWARE
  // Simulated axis: same speed/acceleration limits as the real one.
  void drive_rate(float wanted, float dt) {
    wanted = constrain(wanted, -max_rate_, max_rate_);
    float dv = constrain(wanted - sim_rate_, -max_accel_ * dt, max_accel_ * dt);
    sim_rate_ += dv;
    sim_pos_ += sim_rate_ * dt;
  }
  void drive_position(float target, float dt) { drive_rate(8.0f * (target - sim_pos_), dt); }
  float pos() const { return sim_pos_; }
  float rate() const { return sim_rate_; }
  void set_pos(float p) { sim_pos_ = p; }
  float sim_pos_ = 0.0f, sim_rate_ = 0.0f;
#else
  int32_t to_steps(float rad) const { return (int32_t)lroundf(rad * steps_per_rad_); }
  void drive_rate(float wanted, float) {
    if (fabsf(wanted) < 1e-4f) {
      if (stepper_->isRunning()) stepper_->stopMove();  // decelerate to standstill
      return;
    }
    stepper_->setSpeedInHz((uint32_t)max(1, to_steps(fabsf(wanted))));
    if (wanted > 0) stepper_->runForward();
    else stepper_->runBackward();
  }
  void drive_position(float target, float) {
    stepper_->setSpeedInHz(to_steps(max_rate_));
    stepper_->moveTo(to_steps(target));
    // TODO(TT): use the feed-forward rate from TurretCommand for smoother tracking
  }
  float pos() const { return stepper_->getCurrentPosition() / steps_per_rad_; }
  float rate() const { return stepper_->getCurrentSpeedInMilliHz() / 1000.0f / steps_per_rad_; }
  void set_pos(float p) {
    stepper_->forceStop();
    stepper_->setCurrentPosition(to_steps(p));
  }
  FastAccelStepper *stepper_ = nullptr;
#endif

  const char *name_;
  float steps_per_rad_, min_, max_, max_rate_, max_accel_, switch_pos_;
  uint8_t switch_pin_ = 0;
  uint8_t mode_ = HOLD;
  float target_pos_ = 0.0f, target_rate_ = 0.0f;
  Homing homing_ = Homing::NOT_HOMED;
  uint32_t homing_started_ms_ = 0;
  bool at_limit_ = false;
  bool homing_failed_ = false;
};

Axis yaw("yaw", YAW_GEAR_RATIO, YAW_MIN_RAD, YAW_MAX_RAD, YAW_MAX_RATE, YAW_MAX_ACCEL,
         YAW_HOME_SWITCH_RAD);
Axis pitch("pitch", PITCH_GEAR_RATIO, PITCH_MIN_RAD, PITCH_MAX_RAD, PITCH_MAX_RATE,
           PITCH_MAX_ACCEL, PITCH_HOME_SWITCH_RAD);
uint32_t last_update_ms = 0;
bool was_estop = false;

}  // namespace

void axes_setup() {
#if !SIM_HARDWARE
  engine.init();
#endif
  yaw.begin(PIN_YAW_STEP, PIN_YAW_DIR, YAW_INVERT_DIR, PIN_YAW_HOME_SWITCH);
  pitch.begin(PIN_PITCH_STEP, PIN_PITCH_DIR, PITCH_INVERT_DIR, PIN_PITCH_HOME_SWITCH);
}

void axes_update(uint32_t now_ms, bool motion_allowed, bool estop) {
  if (now_ms - last_update_ms < 10) return;  // 100 Hz
  float dt = (now_ms - last_update_ms) / 1000.0f;
  last_update_ms = now_ms;
  if (estop && !was_estop) {  // motor power was cut: steps may be lost, home again
    yaw.lost_position();
    pitch.lost_position();
  }
  was_estop = estop;
  yaw.update(now_ms, motion_allowed, dt);
  pitch.update(now_ms, motion_allowed, dt);
}

void axes_command(uint8_t mode, float y, float p, float y_rate, float p_rate) {
  yaw.command(mode, y, y_rate);
  pitch.command(mode, p, p_rate);
}

AxisState axes_yaw() { return yaw.state(); }
AxisState axes_pitch() { return pitch.state(); }
