"""target_tracker_node - Stage S4 Tracking and fusion.     OWNER: Sofian

Doc: section 4.2 (S4), section 8.2 (THE important rule), section 12 (latency).

Subscribes:  /vision/detections   DetectionArray
             /range/target        Range
             /turret/state        TurretState
             /camera/camera_info  CameraInfo
Publishes:   /target/state        TargetState   30 Hz (confidence 0 = no target)

Section 8.2 rule, implemented in on_detections():
    bearing (base frame) = turret angle AT THE TIME OF THE IMAGE + angle of target in the image
The turret angle at image time is looked up in a short history of /turret/state.

TODO(Sofian):
  - replace the alpha-beta filter with a Kalman filter (constant velocity model)
  - predict forward by the measured latency (T28)
  - fallback range from ball size in the image when no range sensor is valid
  - camera-to-yaw-axis offset from the URDF / tf (T23)
"""
import bisect
import math
from collections import deque

import rclpy
from pingpong_interfaces.msg import DetectionArray, Range, TargetState, TurretState
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


def seconds(stamp):
    return Time.from_msg(stamp).nanoseconds * 1e-9


class TargetTrackerNode(Node):

    def __init__(self):
        super().__init__('target_tracker_node')
        self.declare_parameter('alpha', 0.6)              # position gain
        self.declare_parameter('beta', 0.1)               # velocity gain
        self.declare_parameter('lost_timeout_s', 0.5)
        self.declare_parameter('range_max_age_s', 0.3)
        self.declare_parameter('min_confidence', 0.3)

        self.fx = self.fy = self.cx = self.cy = None
        self.turret_history = deque(maxlen=200)           # (t, yaw, pitch), ~4 s at 50 Hz
        self.last_range = None

        # filter state
        self.track_id = 0
        self.track_age = 0
        self.last_t = None
        self.yaw = self.pitch = 0.0
        self.yaw_rate = self.pitch_rate = 0.0
        self.confidence = 0.0

        self.pub = self.create_publisher(TargetState, '/target/state', 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.on_info, 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 50)
        self.create_subscription(Range, '/range/target', self.on_range, 10)
        self.create_subscription(DetectionArray, '/vision/detections', self.on_detections, 10)
        self.create_timer(1.0 / 30.0, self.publish_state)

    def on_info(self, msg):
        self.fx, self.fy, self.cx, self.cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]

    def on_turret(self, msg):
        self.turret_history.append((seconds(msg.header.stamp), msg.yaw, msg.pitch))

    def on_range(self, msg):
        self.last_range = msg

    def turret_at(self, t):
        """Turret yaw/pitch at time t, linear interpolation in the history."""
        h = self.turret_history
        times = [s[0] for s in h]
        i = bisect.bisect_left(times, t)
        if i == 0:
            return h[0][1], h[0][2]
        if i >= len(h):
            return h[-1][1], h[-1][2]
        (t0, y0, p0), (t1, y1, p1) = h[i - 1], h[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        return y0 + a * (y1 - y0), p0 + a * (p1 - p0)

    def on_detections(self, msg):
        if self.fx is None or not self.turret_history:
            return
        dets = [d for d in msg.detections
                if d.confidence >= self.get_parameter('min_confidence').value]
        if not dets:
            return
        det = max(dets, key=lambda d: d.confidence)   # TODO(Sofian): data association

        t = seconds(msg.header.stamp)
        turret_yaw, turret_pitch = self.turret_at(t)
        meas_yaw = turret_yaw + math.atan((self.cx - det.u) / self.fx)
        meas_pitch = turret_pitch + math.atan((self.cy - det.v) / self.fy)

        if self.track_age == 0 or self.last_t is None:
            self.track_id += 1
            self.yaw, self.pitch = meas_yaw, meas_pitch
            self.yaw_rate = self.pitch_rate = 0.0
        else:
            dt = max(1e-3, t - self.last_t)
            a = self.get_parameter('alpha').value
            b = self.get_parameter('beta').value
            pred_yaw = self.yaw + self.yaw_rate * dt
            pred_pitch = self.pitch + self.pitch_rate * dt
            r_yaw, r_pitch = meas_yaw - pred_yaw, meas_pitch - pred_pitch
            self.yaw = pred_yaw + a * r_yaw
            self.pitch = pred_pitch + a * r_pitch
            self.yaw_rate += b * r_yaw / dt
            self.pitch_rate += b * r_pitch / dt
        self.last_t = t
        self.track_age += 1
        self.confidence = min(1.0, 0.5 * self.confidence + 0.5 * det.confidence + 0.05)

    def publish_state(self):
        now = self.get_clock().now()
        lost = (self.last_t is None or
                now.nanoseconds * 1e-9 - self.last_t > self.get_parameter('lost_timeout_s').value)
        if lost:
            self.track_age = 0
            self.confidence = 0.0

        msg = TargetState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.yaw_bearing, msg.pitch_bearing = self.yaw, self.pitch
        msg.yaw_rate, msg.pitch_rate = self.yaw_rate, self.pitch_rate
        msg.confidence = float(self.confidence)
        msg.track_id = self.track_id
        msg.track_age = self.track_age

        r = self.last_range
        if r is not None and r.valid and \
                now.nanoseconds * 1e-9 - seconds(r.header.stamp) < self.get_parameter('range_max_age_s').value:
            msg.range, msg.range_valid = float(r.range), True
            cp = math.cos(self.pitch)
            msg.position.x = r.range * cp * math.cos(self.yaw)
            msg.position.y = r.range * cp * math.sin(self.yaw)
            msg.position.z = r.range * math.sin(self.pitch)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TargetTrackerNode())


if __name__ == '__main__':
    main()
