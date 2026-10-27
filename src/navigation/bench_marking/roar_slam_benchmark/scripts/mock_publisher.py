#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math
import time

class MockPublisher(Node):
    def __init__(self):
        super().__init__('mock_publisher')
        
        self.pub_gt = self.create_publisher(PoseWithCovarianceStamped, '/ground_truth/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.start_time = time.time()
        
        self.get_logger().info("Mock Publisher Started. Publishing fake data for 15 seconds...")

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > 15.0:
            self.get_logger().info("Finished publishing mock data. You can now Ctrl+C the slam_evaluator to see the report!")
            raise SystemExit
            
        now = self.get_clock().now()
        
        # 1. Cmd Vel
        cmd_msg = Twist()
        # Drive for 5 seconds, then stop for 10 seconds
        if elapsed < 5.0:
            cmd_msg.linear.x = 1.0
        else:
            cmd_msg.linear.x = 0.0 # Parked!
            
        self.pub_cmd.publish(cmd_msg)
        
        # 2. Ground Truth Pose (moves exactly 1m/s for 5 sec, then stops)
        gt_msg = PoseWithCovarianceStamped()
        gt_msg.header.stamp = now.to_msg()
        gt_msg.header.frame_id = "map"
        
        if elapsed < 5.0:
            gt_x = elapsed * 1.0
        else:
            gt_x = 5.0
            
        gt_msg.pose.pose.position.x = gt_x
        gt_msg.pose.pose.position.y = 0.0
        gt_msg.pose.pose.position.z = 0.0
        self.pub_gt.publish(gt_msg)
        
        # 3. Estimated Odometry (moves 1m/s but with some error, and drifts while parked)
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        
        if elapsed < 5.0:
            est_x = elapsed * 0.95 # Driving with 5% error
            est_y = math.sin(elapsed) * 0.2 # Swaying error
        else:
            # Parked, but we inject "Phantom Drift" of 0.05 m/s
            time_parked = elapsed - 5.0
            est_x = 5.0 * 0.95 + (time_parked * 0.05)
            est_y = math.sin(5.0) * 0.2
            
        odom_msg.pose.pose.position.x = est_x
        odom_msg.pose.pose.position.y = est_y
        odom_msg.pose.pose.position.z = 0.0
        self.pub_odom.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockPublisher()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
