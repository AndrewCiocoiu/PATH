#include "ros_iface.h"

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <pingpong_interfaces/msg/fault.h>
#include <pingpong_interfaces/msg/launcher_command.h>
#include <pingpong_interfaces/msg/launcher_state.h>
#include <pingpong_interfaces/msg/turret_command.h>
#include <pingpong_interfaces/msg/turret_state.h>

#include "axes.h"
#include "config.h"
#include "faults.h"
#include "launcher.h"
#include "safety.h"

#define TS(name) ROSIDL_GET_MSG_TYPE_SUPPORT(pingpong_interfaces, msg, name)

namespace {

// best effort: a lost telemetry message is not worth handling, the next one comes in 20 ms
inline void ignore(rcl_ret_t) {}

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_publisher_t turret_pub, launcher_pub, fault_pub;
rcl_subscription_t turret_sub, launcher_sub;
rcl_timer_t turret_timer, launcher_timer;

pingpong_interfaces__msg__TurretCommand turret_cmd;
pingpong_interfaces__msg__LauncherCommand launcher_cmd;
pingpong_interfaces__msg__TurretState turret_state;
pingpong_interfaces__msg__LauncherState launcher_state;
pingpong_interfaces__msg__Fault fault_msg;

// micro-ROS has no dynamic memory: every string field needs a buffer we own
char turret_cmd_frame[16], launcher_cmd_frame[16], fault_text[48];

void set_string(rosidl_runtime_c__String &s, char *buf, size_t capacity) {
  s.data = buf;
  s.size = 0;
  s.capacity = capacity;
  buf[0] = '\0';
}

void set_const_string(rosidl_runtime_c__String &s, const char *text) {
  s.data = const_cast<char *>(text);
  s.size = strlen(text);
  s.capacity = s.size + 1;
}

// Timestamps in Pi time (synchronised at connect), doc section 2: "everything is timestamped"
void stamp(builtin_interfaces__msg__Time &t) {
  int64_t ns = rmw_uros_epoch_nanos();
  t.sec = ns / 1000000000LL;
  t.nanosec = ns % 1000000000LL;
}

// ------------------------------------------------------------ callbacks
void on_turret_cmd(const void *msg) {
  auto *m = static_cast<const pingpong_interfaces__msg__TurretCommand *>(msg);
  safety_heartbeat(millis());
  axes_command(m->mode, m->yaw, m->pitch, m->yaw_rate, m->pitch_rate);
}

void on_launcher_cmd(const void *msg) {
  auto *m = static_cast<const pingpong_interfaces__msg__LauncherCommand *>(msg);
  launcher_command(m->command, m->seq, millis());
}

void publish_turret(rcl_timer_t *, int64_t) {
  AxisState y = axes_yaw(), p = axes_pitch();
  stamp(turret_state.header.stamp);
  turret_state.yaw = y.pos;
  turret_state.pitch = p.pos;
  turret_state.yaw_rate = y.rate;
  turret_state.pitch_rate = p.rate;
  turret_state.homed = y.homed && p.homed;
  turret_state.at_limit = y.at_limit || p.at_limit;
  turret_state.moving = y.moving || p.moving;
  ignore(rcl_publish(&turret_pub, &turret_state, NULL));
}

void publish_launcher(rcl_timer_t *, int64_t) {
  LauncherInfo l = launcher_info();
  stamp(launcher_state.header.stamp);
  launcher_state.state = l.state;
  launcher_state.ball_present = l.ball_present;
  launcher_state.cocked = l.cocked;
  launcher_state.magazine_empty = l.magazine_empty;
  launcher_state.arm_switch = l.arm_switch;
  launcher_state.shots_fired = l.shots_fired;
  launcher_state.last_cmd_seq = l.last_cmd_seq;
  ignore(rcl_publish(&launcher_pub, &launcher_state, NULL));
}

void publish_faults() {
  FaultEvent f;
  while (fault_pop(f)) {
    stamp(fault_msg.header.stamp);
    fault_msg.code = f.code;
    fault_msg.severity = f.severity;
    fault_msg.active = f.active;
    strncpy(fault_text, f.text, sizeof(fault_text));
    fault_msg.message.size = strlen(fault_text);
    ignore(rcl_publish(&fault_pub, &fault_msg, NULL));
  }
}

}  // namespace

#define CHECK(x) \
  if ((x) != RCL_RET_OK) return false

bool ros_create() {
  allocator = rcl_get_default_allocator();
  CHECK(rclc_support_init(&support, 0, NULL, &allocator));
  CHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

  CHECK(rclc_publisher_init_default(&turret_pub, &node, TS(TurretState), "/turret/state"));
  CHECK(rclc_publisher_init_default(&launcher_pub, &node, TS(LauncherState), "/launcher/state"));
  CHECK(rclc_publisher_init_default(&fault_pub, &node, TS(Fault), "/system/faults"));
  CHECK(rclc_subscription_init_default(&turret_sub, &node, TS(TurretCommand), "/turret/cmd"));
  CHECK(rclc_subscription_init_default(&launcher_sub, &node, TS(LauncherCommand), "/launcher/cmd"));

  CHECK(rclc_timer_init_default2(&turret_timer, &support, RCL_MS_TO_NS(TURRET_STATE_PERIOD_MS),
                                 publish_turret, true));
  CHECK(rclc_timer_init_default2(&launcher_timer, &support,
                                 RCL_MS_TO_NS(LAUNCHER_STATE_PERIOD_MS), publish_launcher, true));

  executor = rclc_executor_get_zero_initialized_executor();
  CHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  CHECK(rclc_executor_add_subscription(&executor, &turret_sub, &turret_cmd, on_turret_cmd, ON_NEW_DATA));
  CHECK(rclc_executor_add_subscription(&executor, &launcher_sub, &launcher_cmd, on_launcher_cmd,
                                       ON_NEW_DATA));
  CHECK(rclc_executor_add_timer(&executor, &turret_timer));
  CHECK(rclc_executor_add_timer(&executor, &launcher_timer));

  set_string(turret_cmd.header.frame_id, turret_cmd_frame, sizeof(turret_cmd_frame));
  set_string(launcher_cmd.header.frame_id, launcher_cmd_frame, sizeof(launcher_cmd_frame));
  set_const_string(turret_state.header.frame_id, "base_link");
  set_const_string(fault_msg.source, "esp32");
  set_string(fault_msg.message, fault_text, sizeof(fault_text));

  rmw_uros_sync_session(1000);  // clock sync with the Pi for timestamps
  return true;
}

void ros_destroy() {
  rmw_context_t *ctx = rcl_context_get_rmw_context(&support.context);
  ignore(rmw_uros_set_context_entity_destroy_session_timeout(ctx, 0));  // agent may be gone
  ignore(rcl_publisher_fini(&turret_pub, &node));
  ignore(rcl_publisher_fini(&launcher_pub, &node));
  ignore(rcl_publisher_fini(&fault_pub, &node));
  ignore(rcl_subscription_fini(&turret_sub, &node));
  ignore(rcl_subscription_fini(&launcher_sub, &node));
  ignore(rcl_timer_fini(&turret_timer));
  ignore(rcl_timer_fini(&launcher_timer));
  ignore(rclc_executor_fini(&executor));
  ignore(rcl_node_fini(&node));
  ignore(rclc_support_fini(&support));
}

void ros_spin() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  if (launcher_changed()) publish_launcher(nullptr, 0);
  publish_faults();
}
