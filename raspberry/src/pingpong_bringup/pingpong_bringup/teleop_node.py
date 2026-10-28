"""teleop_node - manual control from the keyboard.          OWNER: Razvi

Doc: section 5 (teleop_node, manual mode only).

Use ONLY without the autonomy nodes, otherwise two nodes fight over /turret/cmd:
    terminal 1:  ros2 launch pingpong_bringup pingpong.launch.py autonomy:=false
    terminal 2:  ros2 run pingpong_bringup teleop_node

Keys:  a/d yaw   w/s pitch   h home   l load   f FIRE   x stop   q quit
Publishes /turret/cmd at 50 Hz (heartbeat) and /launcher/cmd on key press.
"""
import select
import sys
import termios
import tty

import rclpy
from pingpong_interfaces.msg import LauncherCommand, TurretCommand, TurretState
from rclpy.node import Node

STEP = 0.05   # [rad] per key press
KEYS = 'Keys:  a/d yaw   w/s pitch   h home   l load   f FIRE   x stop   q quit'


class TeleopNode(Node):

    def __init__(self):
        super().__init__('teleop_node')
        self.yaw = self.pitch = None
        self.mode = TurretCommand.HOLD
        self.seq = self.launcher_seq = 0
        self.turret_pub = self.create_publisher(TurretCommand, '/turret/cmd', 10)
        self.launcher_pub = self.create_publisher(LauncherCommand, '/launcher/cmd', 10)
        self.create_subscription(TurretState, '/turret/state', self.on_turret, 10)
        self.create_timer(0.02, self.send)
        print(KEYS)

    def on_turret(self, msg):
        if self.yaw is None:
            self.yaw, self.pitch = msg.yaw, msg.pitch

    def key(self, k):
        if self.yaw is None and k in 'adws':
            return
        if k == 'a':
            self.yaw += STEP
        elif k == 'd':
            self.yaw -= STEP
        elif k == 'w':
            self.pitch += STEP
        elif k == 's':
            self.pitch -= STEP
        elif k == 'h':
            self.mode = TurretCommand.HOME
            self.yaw = self.pitch = 0.0
            return
        elif k == 'x':
            self.mode = TurretCommand.HOLD
            return
        elif k in 'lf':
            self.launcher_seq += 1
            cmd = LauncherCommand()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.command = LauncherCommand.LOAD if k == 'l' else LauncherCommand.FIRE
            cmd.seq = self.launcher_seq
            self.launcher_pub.publish(cmd)
            return
        else:
            return
        self.mode = TurretCommand.POSITION

    def send(self):
        cmd = TurretCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.mode = self.mode
        cmd.yaw, cmd.pitch = self.yaw or 0.0, self.pitch or 0.0
        self.seq += 1
        cmd.seq = self.seq
        self.turret_pub.publish(cmd)


def main():
    rclpy.init()
    node = TeleopNode()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if select.select([sys.stdin], [], [], 0)[0]:
                k = sys.stdin.read(1)
                if k == 'q':
                    break
                node.key(k)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


if __name__ == '__main__':
    main()
