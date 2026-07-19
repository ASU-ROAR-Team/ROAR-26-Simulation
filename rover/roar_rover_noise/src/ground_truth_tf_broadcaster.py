#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class GroundTruthTFBroadcaster(Node):
    def __init__(self):
        super().__init__('ground_truth_tf_broadcaster')
        
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to ground truth odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ground_truth/state',
            self.odom_callback,
            10
        )
        
        self.get_logger().info("Ground Truth TF Broadcaster started")
    
    def odom_callback(self, msg):
        """Convert odometry message to TF transform and broadcast it"""
        # Create transform message
        transform = TransformStamped()
        
        # Header
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id  # "world"
        transform.child_frame_id = msg.child_frame_id    # "base_link"
        
        # Translation
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation
        transform.transform.rotation.x = msg.pose.pose.orientation.x
        transform.transform.rotation.y = msg.pose.pose.orientation.y
        transform.transform.rotation.z = msg.pose.pose.orientation.z
        transform.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    
    broadcaster = GroundTruthTFBroadcaster()
    
    try:
        rclpy.spin(broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        broadcaster.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
