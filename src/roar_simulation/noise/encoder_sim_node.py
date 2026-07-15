#!/usr/bin/env python3
"""
encoder_sim_node.py

Bridges Gazebo's continuous wheel joint angle (radians, from
joint_state_broadcaster) into simulated encoder pulse counts, matching
the units the IESKF node's WIO module expects (it treats incoming
/joint_states 'position' values as raw encoder pulses, not radians).

Subscribes:  /joint_states           (sensor_msgs/JointState, real, radians)
Publishes:   /joint_states_encoded   (sensor_msgs/JointState, simulated pulses)

Then remap the IESKF node at launch time:
    remappings=[('/joint_states', '/joint_states_encoded')]

No changes needed to rover xacros, ieskf_node.cpp, or the real
joint_state_broadcaster output.
"""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EncoderSimNode(Node):
    def __init__(self):
        super().__init__('encoder_sim_node')

        # Pick the representative wheel per side. Mid wheels for now —
        # swap to averaging all three per side later if needed.
        self.left_joint  = self.declare_parameter('left_joint',  'wheel_lhs_mid_joint').value
        self.right_joint = self.declare_parameter('right_joint', 'wheel_rhs_mid_joint').value

        # Matches WIOParams::pulses_per_rev default in ieskf_node.cpp (2000.0).
        # Swap to the real 3531_0-derived value (~201554) once you're ready
        # to tune for hardware accuracy — keeping the default for now so the
        # sim matches what the filter already expects out of the box.
        self.pulses_per_rev = self.declare_parameter('pulses_per_rev', 201554.0).value

        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        self.pub = self.create_publisher(
            JointState, '/joint_states_updated', 10)

        self.get_logger().info(
            f"encoder_sim_node up. left={self.left_joint} right={self.right_joint} "
            f"pulses_per_rev={self.pulses_per_rev}"
        )

    def joint_state_cb(self, msg: JointState):
        left_pos = None
        right_pos = None
        for name, pos in zip(msg.name, msg.position):
            if name == self.left_joint:
                left_pos = pos
            elif name == self.right_joint:
                right_pos = pos

        if left_pos is None or right_pos is None:
            self.get_logger().warn(
                "left/right joint not found in /joint_states yet — skipping",
                throttle_duration_sec=3.0,
            )
            return

        # radians -> simulated pulse count (rounded, so it actually quantizes
        # like a real encoder instead of passing through a perfect float)
        left_pulses  = round(left_pos  / (2.0 * math.pi) * self.pulses_per_rev)
        right_pulses = round(right_pos / (2.0 * math.pi) * self.pulses_per_rev)

        out = JointState()
        out.header = msg.header
        out.name = [self.left_joint, self.right_joint]
        out.position = [float(left_pulses), float(right_pulses)]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderSimNode()
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