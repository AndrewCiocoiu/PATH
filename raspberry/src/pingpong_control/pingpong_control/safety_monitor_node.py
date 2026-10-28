"""safety_monitor_node - watches that every part of the pipeline is alive.    OWNER: Andrei

Doc: section 5 (node graph), section 2 (the ESP32 is safe on its own; this is the Pi-side net).

Subscribes:  the topics in WATCHED (only checks a topic after it has been seen once;
             missing-at-boot is the mission manager's job)
Publishes:   /system/faults   Fault  NODE_SILENT active / cleared
             /operator/arm    Bool   forced False when a fault becomes active
"""
import rclpy
from pingpong_interfaces.msg import Fault, SystemState, TargetState, TurretCommand, TurretState
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool

# topic -> (type, max silence in seconds)
# Looser than the ESP32's own 250 ms watchdog: that one protects the hardware, this one
# reports which part of the Pi pipeline died.
WATCHED = {
    '/camera/camera_info': (CameraInfo, 1.0),
    '/target/state': (TargetState, 1.0),
    '/turret/state': (TurretState, 0.5),
    '/turret/cmd': (TurretCommand, 0.5),
    '/system/state': (SystemState, 1.0),
}


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor_node')
        self.last_seen = {}
        self.silent = set()
        self.fault_pub = self.create_publisher(Fault, '/system/faults', 10)
        self.arm_pub = self.create_publisher(Bool, '/operator/arm', 10)
        for topic, (msg_type, _) in WATCHED.items():
            self.create_subscription(msg_type, topic, lambda _m, t=topic: self.seen(t), 10)
        self.create_timer(0.05, self.check)
        # TODO(Andrei): watch Pi CPU temperature (/sys/class/thermal/thermal_zone0/temp).

    def seen(self, topic):
        self.last_seen[topic] = self.get_clock().now().nanoseconds * 1e-9

    def check(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        for topic, t in self.last_seen.items():
            too_old = now - t > WATCHED[topic][1]
            if too_old and topic not in self.silent:
                self.silent.add(topic)
                self.publish(topic, True)
                self.arm_pub.publish(Bool(data=False))
                self.get_logger().error(f'{topic} silent for more than {WATCHED[topic][1]} s')
            elif not too_old and topic in self.silent:
                self.silent.discard(topic)
                self.publish(topic, False)
                self.get_logger().info(f'{topic} back')

    def publish(self, topic, active):
        f = Fault()
        f.header.stamp = self.get_clock().now().to_msg()
        f.code, f.severity = Fault.NODE_SILENT, Fault.ERROR
        f.source = self.get_name()
        f.message = f'{topic} silent'
        f.active = active
        # all silent topics share one code: only clear when everything is back
        if active or not self.silent:
            self.fault_pub.publish(f)


def main():
    rclpy.init()
    rclpy.spin(SafetyMonitorNode())


if __name__ == '__main__':
    main()
