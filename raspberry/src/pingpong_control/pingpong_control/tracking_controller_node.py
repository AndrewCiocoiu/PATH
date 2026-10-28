"""tracking_controller_node - Stage S7 Motion control (middle loop).    OWNER: Andrei

Doc: section 4.2 (S7), section 8.1 (three nested loops).

Subscribes:  /aim/solution   AimSolution
             /turret/state   TurretState
             /system/state   SystemState  (decides the controller mode)
Publishes:   /turret/cmd     TurretCommand  50 Hz, ALWAYS (it is the ESP32 heartbeat)

Mode per system state:
    HOMING                          -> HOME
    SEARCH                          -> POSITION, scan pattern
    TRACK, AIM, FIRE, VERIFY, RELOAD -> VELOCITY, PI on aim error + velocity feed-forward
    everything else                 -> HOLD
"""
import math

import rclpy
from pingpong_interfaces.msg import AimSolution, SystemState, TurretCommand, TurretState
from rclpy.node import Node

TRACKING_STATES = (SystemState.TRACK, SystemState.AIM, SystemState.FIRE,
                   SystemState.VERIFY, SystemState.RELOAD)


def clamp(x, limit):
    return max(-limit, min(limit, x))


class TrackingControllerNode(Node):

    def __init__(self):
        super().__init__('tracking_controller_node')
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('kp', 4.0)
        self.declare_parameter('ki', 0.5)
        self.declare_parameter('max_rate', 2.0)               # [rad/s]
        self.declare_parameter('dead_band_rad', 0.002)
        self.declare_parameter('search_yaw_amplitude', 1.0)   # [rad]
        self.declare_parameter('search_period_s', 8.0)
        self.declare_parameter('search_pitch', 0.05)          # [rad]

        self.state = SystemState.BOOT
        self.aim = None
        self.turret = None
        self.seq = 0
        self.int_yaw = self.int_pitch = 0.0
        self.prev_aim = None                                  # (t, yaw_cmd, pitch_cmd) for feed-forward
        self.ff_yaw = self.ff_pitch = 0.0
        self.search_t0 = None

        self.pub = self.create_publisher(TurretCommand, '/turret/cmd', 10)
        self.create_subscription(SystemState, '/system/state', self.on_state, 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)
        self.create_subscription(AimSolution, '/aim/solution', self.on_aim, 10)
        self.dt = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(self.dt, self.step)

    def on_state(self, msg):
        if msg.state != self.state:
            self.int_yaw = self.int_pitch = 0.0
            self.search_t0 = None
        self.state = msg.state

    def on_turret(self, msg):
        self.turret = msg

    def on_aim(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_aim is not None and msg.valid:
            dt = now - self.prev_aim[0]
            if dt > 1e-3:
                # low-pass filtered derivative of the aim point = velocity feed-forward
                self.ff_yaw = 0.7 * self.ff_yaw + 0.3 * (msg.yaw_cmd - self.prev_aim[1]) / dt
                self.ff_pitch = 0.7 * self.ff_pitch + 0.3 * (msg.pitch_cmd - self.prev_aim[2]) / dt
        self.prev_aim = (now, msg.yaw_cmd, msg.pitch_cmd) if msg.valid else None
        self.aim = msg

    def step(self):
        cmd = TurretCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = TurretCommand.HOLD

        if self.state == SystemState.HOMING:
            cmd.mode = TurretCommand.HOME
        elif self.state == SystemState.SEARCH:
            cmd.mode = TurretCommand.POSITION
            cmd.yaw, cmd.pitch = self.search_pattern()
        elif self.state in TRACKING_STATES and self.aim is not None and self.aim.valid \
                and self.turret is not None:
            cmd.mode = TurretCommand.VELOCITY
            cmd.yaw_rate = self.pi_axis(self.aim.yaw_cmd - self.turret.yaw, 'int_yaw', self.ff_yaw)
            cmd.pitch_rate = self.pi_axis(self.aim.pitch_cmd - self.turret.pitch, 'int_pitch', self.ff_pitch)
        # TODO(Andrei): tune kp/ki on the real turret (T25); consider POSITION mode for the final aim.

        self.seq += 1
        cmd.seq = self.seq
        self.pub.publish(cmd)

    def pi_axis(self, error, int_name, feed_forward):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        if abs(error) < p('dead_band_rad'):
            error = 0.0
        integ = getattr(self, int_name) + error * self.dt
        integ = clamp(integ, p('max_rate') / max(p('ki'), 1e-6))   # anti wind-up
        setattr(self, int_name, integ)
        return clamp(feed_forward + p('kp') * error + p('ki') * integ, p('max_rate'))

    def search_pattern(self):
        # TODO(Andrei): better pattern (raster with several pitch rows), start from last known target.
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.search_t0 is None:
            self.search_t0 = now
        w = 2 * math.pi / self.get_parameter('search_period_s').value
        yaw = self.get_parameter('search_yaw_amplitude').value * math.sin(w * (now - self.search_t0))
        return yaw, self.get_parameter('search_pitch').value


def main():
    rclpy.init()
    rclpy.spin(TrackingControllerNode())


if __name__ == '__main__':
    main()
