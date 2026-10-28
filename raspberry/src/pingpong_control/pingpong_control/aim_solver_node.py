"""aim_solver_node - Stage S5 Aim solution.              OWNER: Andrei

Doc: section 4.2 (S5), section 8.

Subscribes:  /target/state   TargetState
             /turret/state   TurretState
Publishes:   /aim/solution   AimSolution   (one per TargetState, 30 Hz)

Steps: ballistic elevation from range -> lead for moving targets -> camera-to-barrel correction.
Without a valid range the elevation is computed for default_range_m (good enough to TRACK,
the fire permission blocks firing). This also points the range sensor, which is tilted down
from the barrel, at the target, so a real range arrives.
"""
import math

import rclpy
from pingpong_interfaces.msg import AimSolution, TargetState, TurretState
from rclpy.node import Node

from pingpong_control.ballistics import elevation_angle, time_of_flight


class AimSolverNode(Node):

    def __init__(self):
        super().__init__('aim_solver_node')
        self.declare_parameter('muzzle_speed', 7.9)        # [m/s] TODO(Andrei): measure (T19)
        self.declare_parameter('drag_k', 0.14)             # [1/m] air drag, 0 = no drag (ballistics.py)
        self.declare_parameter('yaw_offset_rad', 0.0)      # barrel vs camera, from calibration (T23)
        self.declare_parameter('pitch_offset_rad', 0.0)
        self.declare_parameter('use_lead', True)
        self.declare_parameter('default_range_m', 2.0)     # [m] used until the range sensor sees the target

        self.turret = None
        self.pub = self.create_publisher(AimSolution, '/aim/solution', 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)
        self.create_subscription(TargetState, '/target/state', self.on_target, 10)

    def on_turret(self, msg):
        self.turret = msg

    def on_target(self, t):
        sol = AimSolution()
        sol.header = t.header
        sol.valid = t.confidence > 0.0
        p = lambda name: self.get_parameter(name).value  # noqa: E731

        yaw, pitch, tof = t.yaw_bearing, t.pitch_bearing, 0.0
        rng = t.range if t.range_valid else p('default_range_m')
        d = rng * math.cos(t.pitch_bearing)
        h = rng * math.sin(t.pitch_bearing)
        elev = elevation_angle(d, h, p('muzzle_speed'), drag_k=p('drag_k'))
        if elev is None:
            sol.valid = False                              # out of reach
        else:
            pitch = elev
            tof = time_of_flight(d, elev, p('muzzle_speed'), drag_k=p('drag_k'))
        if p('use_lead'):
            yaw += t.yaw_rate * tof
            pitch += t.pitch_rate * tof

        sol.yaw_cmd = yaw + p('yaw_offset_rad')
        sol.pitch_cmd = pitch + p('pitch_offset_rad')
        sol.time_of_flight = tof
        if self.turret is not None:
            d_yaw = (sol.yaw_cmd - self.turret.yaw) * math.cos(self.turret.pitch)
            sol.aim_error = math.hypot(d_yaw, sol.pitch_cmd - self.turret.pitch)
        else:
            sol.aim_error, sol.valid = math.inf, False
        self.pub.publish(sol)


def main():
    rclpy.init()
    rclpy.spin(AimSolverNode())


if __name__ == '__main__':
    main()
