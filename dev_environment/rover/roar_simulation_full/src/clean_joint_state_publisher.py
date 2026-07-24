#!/usr/bin/env python3
"""
clean_joint_state_node.py

A simple pass-through node for clean simulation testing.
Subscribes to: /joint_states (raw Gazebo output)
Publishes to:  /joint_states_updated (unified topic for SLAM)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class CleanJointStateNode(Node):
    def __init__(self):
        super().__init__('clean_joint_state_node')

        # Subscribe to the original clean Gazebo data
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 10)
        
        # Publish to the unified SLAM topic
        self.pub = self.create_publisher(
            JointState, '/joint_states_updated', 10)

        self.get_logger().info('Clean Pass-Through Node Started! Routing /joint_states to /joint_states_updated')

    def joint_state_cb(self, msg: JointState):
        # Pass the exact message through to the new topic without any changes
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CleanJointStateNode()
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