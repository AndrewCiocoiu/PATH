"""joint_state_relay - /turret/state -> /joint_states for robot_state_publisher.   OWNER: TT

Doc: section 5 (robot_state_publisher), section 8.3 (frames).
Joint names must match urdf/pingpong.urdf.
"""
import rclpy
from pingpong_interfaces.msg import TurretState
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateRelay(Node):

    def __init__(self):
        super().__init__('joint_state_relay')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)

    def on_turret(self, msg):
        js = JointState()
        js.header.stamp = msg.header.stamp
        js.name = ['yaw_joint', 'pitch_joint']
        js.position = [msg.yaw, msg.pitch]
        js.velocity = [msg.yaw_rate, msg.pitch_rate]
        self.pub.publish(js)


def main():
    rclpy.init()
    rclpy.spin(JointStateRelay())


if __name__ == '__main__':
    main()
