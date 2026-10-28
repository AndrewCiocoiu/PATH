#include "sensors.h"

#include <Arduino.h>

#include "config.h"
#include "launcher.h"

static int32_t counts = 0;
static float speed = 0.0f;  // counts/s
static int32_t speed_counts = 0;
static uint32_t speed_ms = 0;

// speed over 20 ms windows: enough to see the jump when the spring lets go
static void update_speed(uint32_t now_ms) {
  uint32_t dt = now_ms - speed_ms;
  if (dt < 20) return;
  speed = (counts - speed_counts) * 1000.0f / dt;
  speed_counts = counts;
  speed_ms = now_ms;
}

#if SIM_HARDWARE

// Simple model of the slip gear: the spring slows the motor while the toothed half pulls the
// rack, pushes the gear back when the motor coasts, and lets go at COCK_RELEASE_DEG.
static float gear_deg = SIM_GEAR_START_DEG;  // unwrapped
static uint32_t last_ms = 0;

void sensors_setup() {}

void sensors_update(uint32_t now_ms) {
  float dt = (now_ms - last_ms) / 1000.0f;
  last_ms = now_ms;
  float a = fmodf(gear_deg, 360.0f);
  bool loaded = a < COCK_RELEASE_DEG;  // toothed half meshes, spring compressed
  float spring = loaded ? a / COCK_RELEASE_DEG : 0.0f;
  int16_t cmd = launcher_motor_cmd();
  float rate;  // deg/s
  if (launcher_motor_brake()) rate = -2.0f * spring;          // brake: slow creep back
  else if (cmd == 0) rate = -40.0f * spring;                  // coast: spring back-drives
  else rate = cmd / 255.0f * SIM_COCK_RPM * 6.0f * (cmd > 0 ? 1.0f - 0.5f * spring : 1.0f);
  if (loaded && rate < 0 && a + rate * dt < 0) rate = -a / dt;  // rack is on its stop
  gear_deg += rate * dt;
  counts = (int32_t)(gear_deg * COCK_COUNTS_PER_REV / 360.0f);
  update_speed(now_ms);
}

#else

#include <driver/pcnt.h>

// PCNT hardware counter, x4 quadrature. The 16-bit counter wraps at +-PCNT_LIMIT; we read it
// every loop (1 kHz, a few counts per ms) and accumulate the difference.
static constexpr int16_t PCNT_LIMIT = 10000;
static int16_t last_raw = 0;

void sensors_setup() {
  pcnt_config_t c = {};
  c.unit = PCNT_UNIT_0;
  c.counter_h_lim = PCNT_LIMIT;
  c.counter_l_lim = -PCNT_LIMIT;
  c.channel = PCNT_CHANNEL_0;  // edges of A, direction from B
  c.pulse_gpio_num = PIN_COCK_ENC_A;
  c.ctrl_gpio_num = PIN_COCK_ENC_B;
  c.pos_mode = PCNT_COUNT_DEC;
  c.neg_mode = PCNT_COUNT_INC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  pcnt_unit_config(&c);
  c.channel = PCNT_CHANNEL_1;  // edges of B, direction from A
  c.pulse_gpio_num = PIN_COCK_ENC_B;
  c.ctrl_gpio_num = PIN_COCK_ENC_A;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  pcnt_unit_config(&c);
  pcnt_set_filter_value(PCNT_UNIT_0, 100);  // ignore glitches shorter than 1.25 us
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

void sensors_update(uint32_t now_ms) {
  int16_t raw;
  pcnt_get_counter_value(PCNT_UNIT_0, &raw);
  int32_t d = raw - last_raw;
  if (d > PCNT_LIMIT / 2) d -= PCNT_LIMIT;  // counter reset to 0 at a limit
  else if (d < -PCNT_LIMIT / 2) d += PCNT_LIMIT;
  last_raw = raw;
  // TODO(Razvi): check the sign on the real motor; flip here if cocking counts down
  counts += d;
  update_speed(now_ms);
}

#endif

int32_t sensor_cock_counts() { return counts; }
float sensor_cock_speed() { return speed; }
