#!/usr/bin/env python3
# ═════════════════════════════════════════════════════════════════════════════
# wheel_radpersec_multiplier.py — REAL ROVER
# ─────────────────────────────────────────────────────────────────────────────
# cmd_vel_to_wheel_bridge is remapped (in roar_nav2_only.launch.py) to publish
# its wheel commands on /Wheel_RadPerSec_2 instead of its documented default
# /Wheel_RadPerSec (the topic the motor firmware actually consumes). This
# node bridges the two, applying a scale correction along the way.
#
# Input : std_msgs/Float32MultiArray on `input_topic` (default
#         /Wheel_RadPerSec_2), 2-element [right, left] wheel velocities
#         (rad/s), matching cmd_vel_to_wheel_bridge's output layout.
#
# Output: std_msgs/Float32MultiArray on `output_topic` (default
#         /Wheel_RadPerSec), same layout, each element scaled by
#         `multiplier` (default 2.0).
# ═════════════════════════════════════════════════════════════════════════════

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray


class WheelRadPerSecMultiplier(Node):
    def __init__(self):
        super().__init__('wheel_radpersec_multiplier')

        self._input_topic = self.declare_parameter(
            'input_topic', '/Wheel_RadPerSec_2').value
        self._output_topic = self.declare_parameter(
            'output_topic', '/Wheel_RadPerSec').value
        self._multiplier = self.declare_parameter('multiplier', 2.0).value

        wheel_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._sub = self.create_subscription(
            Float32MultiArray, self._input_topic, self._scale_callback, wheel_qos)
        self._pub = self.create_publisher(Float32MultiArray, self._output_topic, wheel_qos)

        self.get_logger().info('Wheel RadPerSec Multiplier started')
        self.get_logger().info(f'  input : {self._input_topic}  (Float32MultiArray)')
        self.get_logger().info(f'  output: {self._output_topic}  (Float32MultiArray)')
        self.get_logger().info(f'  multiplier={self._multiplier}')

    def _scale_callback(self, msg: Float32MultiArray):
        scaled = [value * self._multiplier for value in msg.data]
        self._pub.publish(Float32MultiArray(data=scaled))


def main(args=None):
    rclpy.init(args=args)
    node = WheelRadPerSecMultiplier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
