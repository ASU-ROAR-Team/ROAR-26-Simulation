#!/usr/bin/env python3
"""Relay standard /cmd_vel Twist commands to /diff_drive_controller/cmd_vel_unstamped."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    """Forwards /cmd_vel messages to diff_drive_controller's unstamped velocity topic."""

    def __init__(self):
        super().__init__('cmd_vel_relay')
        self._pub = self.create_publisher(
            Twist, '/diff_drive_controller/cmd_vel_unstamped', 10
        )
        self._sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
