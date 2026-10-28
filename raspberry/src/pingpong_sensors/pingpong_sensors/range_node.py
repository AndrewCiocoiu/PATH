"""range_node - Stage S2 Range measurement on the Pi.            OWNER: Razvi

Doc: section 4.2 (S2). Real robot only (sim:=false); in simulation mock_hw_node publishes
/range/target instead.

Hardware: VL53L1X time-of-flight sensor on the Pi I2C bus (SDA GPIO2, SCL GPIO3, 3.3 V),
mounted 12 deg DOWN from the barrel axis (URDF range_joint): when aiming, the barrel is
5-20 deg above the target, and the narrow cone must still see it. Library: pip install vl53l1x
(Pimoroni).

Publishes:   /range/target   Range   15 Hz   (valid=False when there is no good reading)

Field of view: the sensor sees a cone, 27 deg by default. A 4x4 SPAD region of interest
narrows it to about 15 deg (0.79 m wide at 3 m), still wider than the 20 cm target, so it
measures the target only if the background is far behind it. Test this early (T18).

TODO(Razvi):
  - check the library calls on the real sensor (ROI, timing budget, distance mode)
  - T18: target at 1 / 2 / 3 m with a wall 0.5 m and 2 m behind it
"""
import math

import rclpy
from pingpong_interfaces.msg import Range
from rclpy.node import Node

from pingpong_sensors.range_filter import RangeFilter


class RangeNode(Node):

    def __init__(self):
        super().__init__('range_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x29)
        self.declare_parameter('rate_hz', 15.0)
        self.declare_parameter('distance_mode', 3)        # 1 short, 2 medium, 3 long (up to ~4 m)
        self.declare_parameter('roi_spads', 4)            # 4..16, smaller = narrower cone, less range
        self.declare_parameter('fov_deg', 15.0)           # cone for roi_spads (27 deg at 16)
        self.declare_parameter('window', 5)               # median filter length
        self.declare_parameter('min_m', 0.05)
        self.declare_parameter('max_m', 4.0)

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.filter = RangeFilter(window=p('window'), min_m=p('min_m'), max_m=p('max_m'))
        self.sensor = self.open_sensor()
        self.pub = self.create_publisher(Range, '/range/target', 10)
        self.create_timer(1.0 / p('rate_hz'), self.step)

    def open_sensor(self):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        try:
            import VL53L1X
            tof = VL53L1X.VL53L1X(i2c_bus=p('i2c_bus'), i2c_address=p('i2c_address'))
            tof.open()
            n = p('roi_spads')
            lo, hi = 8 - n // 2, 7 + n // 2                  # centred n x n window of the 16 x 16 array
            tof.set_user_roi(VL53L1X.VL53L1xUserRoi(lo, hi, hi, lo))
            tof.start_ranging(p('distance_mode'))
            self.get_logger().info('VL53L1X ready')
            return tof
        except Exception as e:  # missing library or sensor: keep publishing valid=False
            self.get_logger().error(f'VL53L1X not available ({e}), publishing invalid ranges')
            return None

    def step(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()   # time of measurement
        msg.header.frame_id = 'range_link'
        msg.field_of_view = math.radians(self.get_parameter('fov_deg').value)
        msg.sensor_id = 'vl53l1x'
        reading = None
        if self.sensor is not None:
            mm = self.sensor.get_distance()
            reading = mm / 1000.0 if mm and mm > 0 else None
        msg.range, msg.valid = self.filter.update(reading)
        self.pub.publish(msg)

    def destroy_node(self):
        if self.sensor is not None:
            self.sensor.stop_ranging()
            self.sensor.close()
        super().destroy_node()


def main():
    rclpy.init()
    rclpy.spin(RangeNode())


if __name__ == '__main__':
    main()
