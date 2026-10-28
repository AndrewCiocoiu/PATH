// Project PingPong ESP32 firmware: the "reflexes" (doc section 2).      OWNERS: Razvi, TT
//
// loop() runs two things side by side:
//   1. the real-time part (safety, sensors, launcher, axes): ALWAYS runs, Pi or no Pi
//   2. the micro-ROS link: connects to the agent on the Pi, reconnects after a cable pull
//
// NOTE: Serial (USB) carries micro-ROS. Do NOT Serial.print() for debugging, it breaks the link.
//       Use the LED, a fault message, or a USB-UART adapter on Serial2.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rmw_microros/rmw_microros.h>

#include "axes.h"
#include "config.h"
#include "launcher.h"
#include "ros_iface.h"
#include "safety.h"
#include "sensors.h"

enum class Link { WAITING_AGENT, AGENT_AVAILABLE, CONNECTED, DISCONNECTED };
static Link link_state = Link::WAITING_AGENT;
static uint32_t last_ping_ms = 0;

static void update_link(uint32_t now) {
  switch (link_state) {
    case Link::WAITING_AGENT:
      if (now - last_ping_ms > AGENT_PING_PERIOD_MS) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) link_state = Link::AGENT_AVAILABLE;
      }
      break;
    case Link::AGENT_AVAILABLE:
      if (ros_create()) {
        link_state = Link::CONNECTED;
      } else {
        ros_destroy();
        link_state = Link::WAITING_AGENT;
      }
      break;
    case Link::CONNECTED:
      if (now - last_ping_ms > AGENT_PING_PERIOD_MS) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
          link_state = Link::DISCONNECTED;
          break;
        }
      }
      ros_spin();
      break;
    case Link::DISCONNECTED:
      ros_destroy();
      link_state = Link::WAITING_AGENT;
      break;
  }
}

// LED: slow blink = waiting for the Pi, on = connected, fast blink = e-stop
static void update_led(uint32_t now) {
  bool on;
  if (safety_estop()) on = (now / 100) % 2;
  else if (link_state == Link::CONNECTED) on = true;
  else on = (now / 500) % 2;
  digitalWrite(PIN_LED, on);
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(ROS_SERIAL_BAUD);
  set_microros_serial_transports(Serial);

  safety_setup();
  sensors_setup();
  launcher_setup();
  axes_setup();
}

void loop() {
  uint32_t now = millis();
  safety_update(now);
  sensors_update(now);
  launcher_update(now);
  axes_update(now, safety_motion_allowed(), safety_estop());
  update_link(now);
  update_led(now);
}
