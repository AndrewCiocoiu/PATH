"""mock_hw_node - fake ESP32 + fake world, for testing without hardware.   OWNER: Razvi

Doc: section 5 (mock_hw_node), behaves like stages S9-S10 and section 7/11.

Same topics as the real ESP32 firmware (esp32/), so nothing else knows the difference:
    Subscribes:  /turret/cmd, /launcher/cmd
    Publishes:   /turret/state (50 Hz), /launcher/state (on change + 5 Hz),
                 /range/target (15 Hz), /system/faults
Extra for the synthetic camera:
    Publishes:   /sim/target_truth  geometry_msgs/PointStamped, where the simulated ball target is

Simulates: axis speed/acceleration limits, soft limits, homing, 250 ms heartbeat
watchdog, ESP32-side fire interlocks, and the slip gear launcher like esp32/src/launcher.cpp:
gear homing once (arm switch ON), LOAD = cock to the hold + a ball drops in (no feeder, no
ball sensor), balls counted, RESET = holder refilled. /range/target stands in for range_node.

Play with it at runtime, e.g.:
    ros2 param set /mock_hw_node estop true          # press the emergency stop
    ros2 param set /mock_hw_node arm_switch false    # flip the hardware arm switch
    ros2 param set /mock_hw_node target_motion_amplitude 0.0   # static target
"""
import math
import random

import rclpy
from geometry_msgs.msg import PointStamped
from pingpong_interfaces.msg import (Fault, LauncherCommand, LauncherState, Range,
                                     TurretCommand, TurretState)
from rclpy.node import Node

L = LauncherState
DT = 0.02   # 50 Hz


class Axis:
    def __init__(self, pos, lo, hi, max_rate, max_acc):
        self.pos, self.rate = pos, 0.0
        self.lo, self.hi, self.max_rate, self.max_acc = lo, hi, max_rate, max_acc

    def step(self, wanted_rate):
        wanted_rate = max(-self.max_rate, min(self.max_rate, wanted_rate))
        dv = max(-self.max_acc * DT, min(self.max_acc * DT, wanted_rate - self.rate))
        self.rate += dv
        self.pos += self.rate * DT
        at_limit = not self.lo <= self.pos <= self.hi
        if at_limit:
            self.pos, self.rate = max(self.lo, min(self.hi, self.pos)), 0.0
        return at_limit

    def rate_to(self, target, gain=8.0):
        return gain * (target - self.pos)


class MockHwNode(Node):

    def __init__(self):
        super().__init__('mock_hw_node')
        self.declare_parameter('estop', False)
        self.declare_parameter('arm_switch', True)
        self.declare_parameter('magazine_balls', 5)              # holder full at start (after RESET)
        self.declare_parameter('target_range', 2.5)                # [m]
        self.declare_parameter('target_yaw', 0.4)                  # [rad] centre of target motion
        self.declare_parameter('target_pitch', 0.05)
        self.declare_parameter('target_motion_amplitude', 0.15)    # [rad]
        self.declare_parameter('target_motion_period_s', 10.0)
        self.declare_parameter('range_fov_deg', 15.0)              # VL53L1X cone (range_node)
        self.declare_parameter('range_tilt_deg', 12.0)             # sensor axis below the barrel (URDF)

        self.yaw = Axis(random.uniform(-0.5, 0.5), -2.8, 2.8, 2.0, 8.0)
        self.pitch = Axis(random.uniform(-0.2, 0.2), -0.35, 0.8, 1.5, 6.0)
        self.homed = False
        self.at_limit = False
        self.last_cmd = None
        self.last_cmd_time = -1e9
        self.watchdog_tripped = False
        self.estop_reported = False

        self.ls = LauncherState()
        self.ls.state = L.EMPTY
        self.balls = self.get_parameter('magazine_balls').value
        self.gear_homed = False
        self.gear_homing_timer = 0.0
        self.unloading = False
        self.launcher_timer = 0.0        # time left in the current timed launcher state
        self.last_launcher_seq = 0
        self.last_published_launcher = None

        self.turret_pub = self.create_publisher(TurretState, '/turret/state', 10)
        self.launcher_pub = self.create_publisher(LauncherState, '/launcher/state', 10)
        self.range_pub = self.create_publisher(Range, '/range/target', 10)
        self.fault_pub = self.create_publisher(Fault, '/system/faults', 10)
        self.truth_pub = self.create_publisher(PointStamped, '/sim/target_truth', 10)
        self.create_subscription(TurretCommand, '/turret/cmd', self.on_turret_cmd, 10)
        self.create_subscription(LauncherCommand, '/launcher/cmd', self.on_launcher_cmd, 10)
        self.create_timer(DT, self.step)
        self.create_timer(1.0 / 15.0, self.publish_range)
        self.create_timer(0.2, lambda: self.publish_launcher(force=True))
        self.get_logger().info('mock_hw_node: simulated ESP32 running')

    # ------------------------------------------------------------ helpers
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def param(self, name):
        return self.get_parameter(name).value

    def fault(self, code, severity, active, text):
        f = Fault()
        f.header.stamp = self.get_clock().now().to_msg()
        f.code, f.severity, f.active, f.source, f.message = code, severity, active, 'esp32(mock)', text
        self.fault_pub.publish(f)

    def heartbeat_ok(self):
        return self.now() - self.last_cmd_time < 0.25

    def target_bearing(self):
        t = self.now()
        w = 2 * math.pi / self.param('target_motion_period_s')
        yaw = self.param('target_yaw') + self.param('target_motion_amplitude') * math.sin(w * t)
        return yaw, self.param('target_pitch')

    # ------------------------------------------------------------ commands
    def on_turret_cmd(self, msg):
        self.last_cmd, self.last_cmd_time = msg, self.now()

    def on_launcher_cmd(self, msg):
        if msg.seq == self.last_launcher_seq:
            return                                   # duplicate: never fire twice
        self.last_launcher_seq = msg.seq
        self.ls.last_cmd_seq = msg.seq
        c = msg.command
        if c in (LauncherCommand.LOAD, LauncherCommand.COCK) and self.ls.state in (L.EMPTY, L.FIRED):
            if self.balls <= 0:
                self.fault(Fault.FIRE_REFUSED, Fault.WARNING, True, 'LOAD refused: magazine empty')
            elif not self.gear_homed:
                self.fault(Fault.FIRE_REFUSED, Fault.WARNING, True, 'LOAD refused: gear not homed')
            else:
                self.ls.state, self.launcher_timer = L.COCKING, 0.5      # cock to the hold
        elif c == LauncherCommand.FIRE:
            refused = []
            if self.ls.state != L.READY:
                refused.append('not READY')
            elif not self.ls.ball_present:
                refused.append('no ball (magazine)')
            if not self.param('arm_switch'):
                refused.append('arm switch OFF')
            if self.param('estop'):
                refused.append('estop')
            if not self.heartbeat_ok():
                refused.append('no heartbeat')
            if refused:
                self.fault(Fault.FIRE_REFUSED, Fault.WARNING, True, 'FIRE refused: ' + ', '.join(refused))
            else:
                self.ls.state, self.launcher_timer = L.FIRING, 0.07
        elif c == LauncherCommand.UNLOAD and self.ls.state == L.READY:
            self.ls.state, self.launcher_timer, self.unloading = L.COCKING, 0.5, True
        elif c == LauncherCommand.RESET:
            if self.ls.state == L.JAM:
                self.ls.state = L.EMPTY
            self.balls = self.param('magazine_balls')                   # holder refilled

    # ------------------------------------------------------------ simulation
    def step(self):
        estop = self.param('estop')
        if estop != self.estop_reported:
            self.estop_reported = estop
            self.fault(Fault.ESTOP, Fault.FATAL, estop, 'emergency stop')

        hb = self.heartbeat_ok()
        if not hb and not self.watchdog_tripped and self.last_cmd is not None:
            self.watchdog_tripped = True
            self.fault(Fault.HEARTBEAT_LOST, Fault.ERROR, True, 'no TurretCommand for 250 ms')
        elif hb and self.watchdog_tripped:
            self.watchdog_tripped = False
            self.fault(Fault.HEARTBEAT_LOST, Fault.ERROR, False, 'heartbeat back')

        yaw_rate = pitch_rate = 0.0
        cmd = self.last_cmd
        if cmd is not None and hb and not estop:
            if cmd.mode == TurretCommand.HOME:
                # move slowly to the "switch" at 0, 0
                yaw_rate = max(-0.4, min(0.4, self.yaw.rate_to(0.0, 2.0)))
                pitch_rate = max(-0.4, min(0.4, self.pitch.rate_to(0.0, 2.0)))
                if abs(self.yaw.pos) < 0.002 and abs(self.pitch.pos) < 0.002:
                    self.homed = True
            elif self.homed and cmd.mode == TurretCommand.POSITION:
                yaw_rate = self.yaw.rate_to(cmd.yaw) + cmd.yaw_rate
                pitch_rate = self.pitch.rate_to(cmd.pitch) + cmd.pitch_rate
            elif self.homed and cmd.mode == TurretCommand.VELOCITY:
                yaw_rate, pitch_rate = cmd.yaw_rate, cmd.pitch_rate
        at_limit = self.yaw.step(yaw_rate) | self.pitch.step(pitch_rate)
        if at_limit != self.at_limit:
            self.at_limit = at_limit
            self.fault(Fault.AT_LIMIT, Fault.WARNING, at_limit, 'soft limit')

        self.step_launcher()

        ts = TurretState()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        ts.yaw, ts.pitch = self.yaw.pos, self.pitch.pos
        ts.yaw_rate, ts.pitch_rate = self.yaw.rate, self.pitch.rate
        ts.homed, ts.at_limit = self.homed, at_limit
        ts.moving = abs(self.yaw.rate) > 1e-3 or abs(self.pitch.rate) > 1e-3
        self.turret_pub.publish(ts)

        yaw, pitch = self.target_bearing()
        r = self.param('target_range')
        truth = PointStamped()
        truth.header = ts.header
        truth.point.x = r * math.cos(pitch) * math.cos(yaw)
        truth.point.y = r * math.cos(pitch) * math.sin(yaw)
        truth.point.z = r * math.sin(pitch)
        self.truth_pub.publish(truth)

    def step_launcher(self):
        s = self.ls
        # gear homing once: one slow turn through a release, needs the arm switch
        if not self.gear_homed and s.state == L.EMPTY and self.param('arm_switch') \
                and self.heartbeat_ok() and not self.param('estop'):
            self.gear_homing_timer += DT
            if self.gear_homing_timer > 1.5:
                self.gear_homed = True
        if self.launcher_timer > 0:
            self.launcher_timer -= DT
            if self.launcher_timer <= 0:
                if s.state == L.COCKING and self.unloading:
                    s.state, s.ball_present, self.unloading = L.EMPTY, False, False
                elif s.state == L.COCKING:                         # at the hold, ball dropping
                    s.ball_present = self.balls > 0
                    self.balls = max(0, self.balls - 1)
                    s.state, self.launcher_timer = L.LOADING, 0.15
                elif s.state == L.LOADING:
                    s.state = L.READY
                elif s.state == L.FIRING:
                    s.state = L.FIRED
                    s.shots_fired += 1
                    s.ball_present = False
        s.cocked = s.state in (L.LOADING, L.READY)
        s.magazine_empty = self.balls <= 0
        s.arm_switch = self.param('arm_switch')
        self.publish_launcher()

    def publish_launcher(self, force=False):
        s = self.ls
        key = (s.state, s.ball_present, s.cocked, s.magazine_empty, s.arm_switch, s.shots_fired, s.last_cmd_seq)
        if force or key != self.last_published_launcher:
            self.last_published_launcher = key
            s.header.stamp = self.get_clock().now().to_msg()
            self.launcher_pub.publish(s)

    def publish_range(self):
        yaw, pitch = self.target_bearing()
        r = self.param('target_range')
        # While aiming the barrel is 5-20 deg above the target (ballistic elevation), so the
        # sensor is tilted down (range_tilt_deg) to keep the target inside its cone.
        beam = math.radians(self.param('range_fov_deg')) / 2 + math.atan(0.1 / r)  # + 20 cm target
        axis_pitch = self.pitch.pos - math.radians(self.param('range_tilt_deg'))
        m = Range()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'range_link'
        m.field_of_view = math.radians(self.param('range_fov_deg'))
        m.sensor_id = 'mock'
        m.valid = abs(yaw - self.yaw.pos) < beam and abs(pitch - axis_pitch) < beam
        m.range = self.param('target_range') + random.gauss(0.0, 0.02) if m.valid else 0.0
        self.range_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(MockHwNode())


if __name__ == '__main__':
    main()
