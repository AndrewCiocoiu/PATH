"""mission_manager_node - Stage S6 Mission decision (outer loop).    OWNER: Andrei

Doc: section 9 (process line + state machine table), section 10 (fire permission),
     section 14 (shot log).

Subscribes:  /target/state, /aim/solution, /turret/state, /launcher/state,
             /system/faults, /operator/arm (std_msgs/Bool), /operator/reset (std_msgs/Empty)
Publishes:   /system/state    SystemState     20 Hz
             /launcher/cmd    LauncherCommand on events

How to read this file: one method per state, named state_<NAME>(). Each returns the
next state (or None to stay). Rules that apply to every state are in step().
The table in doc section 9.2 is the specification: keep them in sync.
"""
import json
import math

import rclpy
from pingpong_interfaces.msg import (AimSolution, Fault, LauncherCommand, LauncherState,
                                     SystemState, TargetState, TurretState)
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Empty

from pingpong_control.fire_permission import FireConfig, FireInputs, check

S = SystemState
NAMES = {getattr(S, n): n for n in ('BOOT', 'SELF_TEST', 'HOMING', 'IDLE', 'SEARCH', 'TRACK', 'AIM',
                                    'FIRE', 'VERIFY', 'RELOAD', 'FAULT', 'ESTOP')}
ARMED_STATES = (S.SEARCH, S.TRACK, S.AIM, S.FIRE, S.VERIFY, S.RELOAD)


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')
        self.declare_parameter('boot_timeout_s', 30.0)
        self.declare_parameter('homing_timeout_s', 20.0)
        self.declare_parameter('confirm_frames', 5)           # "seen in N of M frames"
        self.declare_parameter('target_lost_s', 1.0)
        self.declare_parameter('aim_timeout_s', 3.0)
        self.declare_parameter('fire_timeout_s', 1.0)
        self.declare_parameter('heartbeat_timeout_s', 0.25)
        fc = FireConfig()
        for name, value in vars(fc).items():
            if name != 'no_fire_zones':
                self.declare_parameter('fire.' + name, value)

        self.state = S.BOOT
        self.state_since = self.now()
        self.message = ''
        self.sw_armed = False
        self.reset_requested = False
        self.faults = {}                   # code -> Fault (active ones only)
        self.target = TargetState()
        self.target_seen_at = -1e9
        self.aim = None
        self.aim_ok_cycles = 0
        self.turret = None
        self.turret_at = -1e9
        self.launcher = None
        self.camera_at = -1e9
        self.last_shot_at = -1e9
        self.launcher_seq = 0
        self.last_load_at = -1e9
        self.shots_at_fire = 0

        self.state_pub = self.create_publisher(SystemState, '/system/state', 10)
        self.launcher_pub = self.create_publisher(LauncherCommand, '/launcher/cmd', 10)
        self.create_subscription(TargetState, '/target/state', self.on_target, 10)
        self.create_subscription(AimSolution, '/aim/solution', self.on_aim, 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)
        self.create_subscription(LauncherState, '/launcher/state', self.on_launcher, 10)
        self.create_subscription(Fault, '/system/faults', self.on_fault, 50)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_camera, 10)
        self.create_subscription(Bool, '/operator/arm', self.on_arm, 10)
        self.create_subscription(Empty, '/operator/reset', self.on_reset, 10)
        self.create_timer(0.05, self.step)

    # ---------------------------------------------------------------- inputs
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_target(self, msg):
        self.target = msg
        if msg.confidence > 0.0:
            self.target_seen_at = self.now()

    def on_aim(self, msg):
        self.aim = msg
        max_err = self.get_parameter('fire.max_aim_error_rad').value
        self.aim_ok_cycles = self.aim_ok_cycles + 1 if msg.valid and msg.aim_error < max_err else 0

    def on_turret(self, msg):
        self.turret, self.turret_at = msg, self.now()

    def on_launcher(self, msg):
        self.launcher = msg

    def on_camera(self, _msg):
        self.camera_at = self.now()

    def on_fault(self, msg):
        if msg.active:
            self.faults[msg.code] = msg
        else:
            self.faults.pop(msg.code, None)

    def on_arm(self, msg):
        self.sw_armed = msg.data

    def on_reset(self, _msg):
        self.reset_requested = True

    # ---------------------------------------------------------------- helpers
    def hw_armed(self):
        return self.launcher is not None and self.launcher.arm_switch

    def armed(self):
        return self.sw_armed and self.hw_armed()

    def estop(self):
        return Fault.ESTOP in self.faults

    def error_fault(self):
        return any(f.severity >= Fault.ERROR and f.code != Fault.ESTOP for f in self.faults.values())

    def heartbeat_ok(self):
        return self.now() - self.turret_at < self.get_parameter('heartbeat_timeout_s').value

    def in_state_for(self):
        return self.now() - self.state_since

    def target_lost(self):
        return self.now() - self.target_seen_at > self.get_parameter('target_lost_s').value

    def launcher_is(self, state):
        return self.launcher is not None and self.launcher.state == state

    def send_launcher(self, command):
        self.launcher_seq += 1
        msg = LauncherCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command, msg.seq = command, self.launcher_seq
        self.launcher_pub.publish(msg)

    def raise_fault(self, code, text):
        f = Fault()
        f.header.stamp = self.get_clock().now().to_msg()
        f.code, f.severity, f.source, f.message, f.active = code, Fault.ERROR, self.get_name(), text, True
        self.faults[code] = f
        self.get_logger().error(text)
        return S.FAULT

    def fire_permission(self):
        p = lambda name: self.get_parameter('fire.' + name).value  # noqa: E731
        cfg = FireConfig(**{k: p(k) for k in vars(FireConfig()) if k != 'no_fire_zones'})
        # TODO(Andrei): load no_fire_zones from the parameter file.
        t, tu = self.target, self.turret
        inputs = FireInputs(
            state_is_aim=self.state == S.AIM,
            software_armed=self.sw_armed,
            hardware_armed=self.hw_armed(),
            estop=self.estop(),
            fault_active=self.error_fault(),
            heartbeat_ok=self.heartbeat_ok(),
            target_confidence=t.confidence,
            track_age=t.track_age,
            aim_ok_cycles=self.aim_ok_cycles,
            range_valid=t.range_valid,
            range_m=t.range,
            turret_rate=max(abs(tu.yaw_rate), abs(tu.pitch_rate)) if tu else 0.0,
            target_rate=max(abs(t.yaw_rate), abs(t.pitch_rate)),
            launcher_ready=self.launcher_is(LauncherState.READY),
            barrel_yaw=tu.yaw if tu else 0.0,
            barrel_pitch=tu.pitch if tu else 0.0,
            time_since_last_shot=self.now() - self.last_shot_at,
        )
        return check(inputs, cfg)

    # ---------------------------------------------------------------- states
    def state_BOOT(self):
        if self.heartbeat_ok() and self.launcher is not None and self.now() - self.camera_at < 1.0:
            return S.SELF_TEST
        if self.in_state_for() > self.get_parameter('boot_timeout_s').value:
            return self.raise_fault(Fault.BOOT_TIMEOUT, 'boot timeout: nodes or ESP32 link missing')
        self.message = 'waiting for camera, turret state and launcher state'

    def state_SELF_TEST(self):
        # TODO(Andrei): Pi temperature, range sensor valid, switches plausible (doc P2).
        problems = []
        if self.now() - self.camera_at > 1.0:
            problems.append('no camera images')
        if not self.heartbeat_ok():
            problems.append('ESP32 link down')
        if problems:
            return self.raise_fault(Fault.SELF_TEST_FAILED, 'self-test failed: ' + ', '.join(problems))
        return S.HOMING

    def state_HOMING(self):
        # the tracking controller sends HOME while we are in this state
        if self.turret is not None and self.turret.homed:
            return S.IDLE
        if self.in_state_for() > self.get_parameter('homing_timeout_s').value:
            return self.raise_fault(Fault.HOMING_TIMEOUT, 'homing timeout')
        self.message = 'homing'

    def state_IDLE(self):
        if self.launcher_is(LauncherState.EMPTY) and not self.launcher.magazine_empty \
                and self.now() - self.last_load_at > 2.0:
            self.last_load_at = self.now()
            self.send_launcher(LauncherCommand.LOAD)          # retried every 2 s while EMPTY
        if self.launcher is not None and self.launcher.magazine_empty:
            # no ball sensor: the ESP32 counts balls, RESET tells it the holder was filled
            if self.reset_requested:
                self.reset_requested = False
                self.send_launcher(LauncherCommand.RESET)
            self.message = 'magazine empty: fill the holder, then /operator/reset'
        elif not self.armed():
            self.message = 'disarmed' + ('' if self.hw_armed() else ' (hardware arm switch OFF)')
        elif self.launcher_is(LauncherState.READY):
            return S.SEARCH
        else:
            self.message = 'armed, waiting for launcher READY'

    def state_SEARCH(self):
        if self.target.confidence > 0 and \
                self.target.track_age >= self.get_parameter('confirm_frames').value:
            return S.TRACK
        self.message = 'searching'

    def state_TRACK(self):
        if self.target_lost():
            return S.SEARCH
        if self.target.track_age >= self.get_parameter('fire.min_track_age').value \
                and self.target.range_valid:
            return S.AIM
        self.message = 'tracking' + ('' if self.target.range_valid else ', no valid range')

    def state_AIM(self):
        if self.target_lost():
            return S.SEARCH
        failed = self.fire_permission()
        if not failed:
            return S.FIRE
        if self.in_state_for() > self.get_parameter('aim_timeout_s').value:
            return S.TRACK
        self.message = 'aiming, waiting for: ' + failed[0]

    def state_FIRE(self):
        if self.launcher is not None and self.launcher.shots_fired > self.shots_at_fire:
            self.last_shot_at = self.now()
            return S.VERIFY
        if self.in_state_for() > self.get_parameter('fire_timeout_s').value:
            return self.raise_fault(Fault.FIRE_TIMEOUT, 'no FIRED within timeout')
        self.message = 'firing'

    def state_VERIFY(self):
        # doc section 14: one log line per shot. TODO(Andrei): hit/miss from operator or camera.
        t, a, tu = self.target, self.aim, self.turret
        self.get_logger().info('SHOT ' + json.dumps({
            'time': self.now(), 'shot': self.launcher.shots_fired,
            'target': {'yaw': t.yaw_bearing, 'pitch': t.pitch_bearing, 'range': t.range,
                       'conf': t.confidence, 'track_age': t.track_age},
            'aim': {'yaw': a.yaw_cmd, 'pitch': a.pitch_cmd, 'error_deg': math.degrees(a.aim_error),
                    'tof': a.time_of_flight} if a else None,
            'turret': {'yaw': tu.yaw, 'pitch': tu.pitch} if tu else None,
        }))
        return S.RELOAD

    def state_RELOAD(self):
        if self.launcher.magazine_empty and not self.launcher.ball_present:
            return S.IDLE
        if self.launcher_is(LauncherState.JAM):
            return self.raise_fault(Fault.LAUNCHER_JAM, 'launcher jam')
        if self.launcher_is(LauncherState.READY):
            return S.SEARCH if self.target_lost() else S.TRACK
        self.message = 'reloading'

    def state_FAULT(self):
        if self.reset_requested:
            # faults raised by this node are cleared by the reset; others must clear at their source
            self.faults = {c: f for c, f in self.faults.items() if f.source != self.get_name()}
            if not self.error_fault():
                return S.IDLE if self.turret is not None and self.turret.homed else S.HOMING
            self.reset_requested = False
        if self.error_fault():
            self.message = 'FAULT: ' + '; '.join(f.message for f in self.faults.values())
        else:
            self.message = 'fault cleared, waiting for operator reset (/operator/reset)'

    def state_ESTOP(self):
        if not self.estop() and self.reset_requested:
            return S.HOMING
        self.message = 'EMERGENCY STOP' if self.estop() else 'e-stop released, waiting for operator reset'

    # ---------------------------------------------------------------- main loop
    def on_enter(self, state):
        if state == S.FIRE:
            self.shots_at_fire = self.launcher.shots_fired
            self.send_launcher(LauncherCommand.FIRE)
        elif state == S.RELOAD:
            self.send_launcher(LauncherCommand.LOAD)
        elif state in (S.FAULT, S.ESTOP):
            self.sw_armed = False

    def step(self):
        # rules for every state (doc 9.2, last row)
        if self.estop() and self.state != S.ESTOP:
            nxt = S.ESTOP
        elif self.error_fault() and self.state not in (S.FAULT, S.ESTOP):
            nxt = S.FAULT
        elif self.state in ARMED_STATES and not self.armed():
            nxt = S.IDLE
        else:
            nxt = getattr(self, 'state_' + NAMES[self.state])()

        if nxt is not None and nxt != self.state:
            self.get_logger().info(f'{NAMES[self.state]} -> {NAMES[nxt]}')
            self.state, self.state_since = nxt, self.now()
            self.reset_requested = False
            self.on_enter(nxt)

        msg = SystemState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state, msg.state_name = self.state, NAMES[self.state]
        msg.armed, msg.estop = self.armed(), self.estop()
        active = [c for c, f in self.faults.items() if f.severity >= Fault.ERROR]
        msg.fault_code = active[0] if active else 0
        msg.message = self.message
        self.state_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(MissionManagerNode())


if __name__ == '__main__':
    main()
