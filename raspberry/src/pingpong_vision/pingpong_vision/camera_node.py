"""camera_node - Stage S1 Image capture.            OWNER: Sofian

Doc: section 4.2 (S1), section 13 (camera is an open decision).

Publishes:
    /camera/image_raw     sensor_msgs/Image       30 Hz, stamp = time of capture
    /camera/camera_info   sensor_msgs/CameraInfo

Parameter `source`:
    'synthetic' (default) draws an orange ball on a grey background, seen from the
                current turret angle. Lets the whole pipeline run without hardware.
                Needs /sim/target_truth and /turret/state from mock_hw_node.
    'opencv'    real camera through cv2.VideoCapture.   <-- TODO(Sofian)

When the camera is decided, only this file and its calibration change.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from pingpong_interfaces.msg import TurretState
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter('source', 'synthetic')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('horizontal_fov_deg', 66.0)   # Pi Camera Module 3 standard
        self.declare_parameter('ball_radius_m', 0.02)        # 40 mm ball

        self.source = self.get_parameter('source').value
        self.w = self.get_parameter('width').value
        self.h = self.get_parameter('height').value
        hfov = math.radians(self.get_parameter('horizontal_fov_deg').value)
        self.fx = (self.w / 2.0) / math.tan(hfov / 2.0)
        self.cx, self.cy = self.w / 2.0, self.h / 2.0

        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        if self.source == 'synthetic':
            self.turret_yaw = 0.0
            self.turret_pitch = 0.0
            self.target = None
            self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)
            self.create_subscription(PointStamped, '/sim/target_truth', self.on_target, 10)
            self.background = np.full((self.h, self.w, 3), 90, dtype=np.uint8)
            self.yy, self.xx = np.mgrid[0:self.h, 0:self.w]
        elif self.source == 'opencv':
            # TODO(Sofian): open the real camera.
            #   import cv2
            #   self.cap = cv2.VideoCapture(self.get_parameter('device').value)
            #   set width/height/fps, FIXED exposure (auto exposure ruins colour detection)
            raise NotImplementedError("source:=opencv is not implemented yet (Sofian)")
        else:
            raise ValueError(f'unknown source {self.source}')

        self.create_timer(1.0 / self.get_parameter('fps').value, self.capture)
        self.get_logger().info(f'camera_node started, source={self.source}')

    def on_turret(self, msg):
        self.turret_yaw, self.turret_pitch = msg.yaw, msg.pitch

    def on_target(self, msg):
        self.target = msg.point

    def capture(self):
        stamp = self.get_clock().now().to_msg()   # time of capture: the tracker relies on this
        if self.source == 'synthetic':
            frame = self.render_synthetic()
        else:
            # TODO(Sofian): ok, frame = self.cap.read(); convert BGR -> RGB
            return

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = 'camera_optical'
        img.height, img.width = frame.shape[:2]
        img.encoding = 'rgb8'
        img.step = img.width * 3
        img.data = frame.tobytes()
        self.image_pub.publish(img)
        self.info_pub.publish(self.camera_info(stamp))

    def render_synthetic(self):
        frame = self.background.copy()
        if self.target is None:
            return frame
        t = self.target
        dist = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        d_yaw = math.atan2(t.y, t.x) - self.turret_yaw
        d_pitch = math.atan2(t.z, math.hypot(t.x, t.y)) - self.turret_pitch
        if abs(d_yaw) > 1.4 or abs(d_pitch) > 1.4:
            return frame
        u = self.cx - self.fx * math.tan(d_yaw)      # target to the left -> smaller u
        v = self.cy - self.fx * math.tan(d_pitch)    # target higher -> smaller v
        r = max(2.0, self.fx * self.get_parameter('ball_radius_m').value / dist)
        mask = (self.xx - u) ** 2 + (self.yy - v) ** 2 <= r ** 2
        frame[mask] = (255, 120, 0)                   # orange
        return frame

    def camera_info(self, stamp):
        # TODO(Sofian): load the real calibration (T16) instead of this ideal pinhole model.
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera_optical'
        info.width, info.height = self.w, self.h
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [self.fx, 0.0, self.cx, 0.0, self.fx, self.cy, 0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0, 0.0, self.fx, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info


def main():
    rclpy.init()
    rclpy.spin(CameraNode())


if __name__ == '__main__':
    main()
