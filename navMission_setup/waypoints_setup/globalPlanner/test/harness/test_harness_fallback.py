#!/usr/bin/env python3
import os
import sys
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, DurabilityPolicy

class TestHarnessFallback(Node):
    def __init__(self):
        super().__init__('test_harness_fallback')
        
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('timeout_sec', 60.0)
        
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        self.get_logger().info("Test Harness Fallback started...")
        self.get_logger().info(f"Target Unreachable Goal: ({self.goal_x}, {self.goal_y})")
        
        self.map_msg = None
        self.odom_msg = None
        self.last_plan = None
        self.action_done = False
        self.action_success = False
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/global_plan', self.plan_callback, 10)
        
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.5, self.loop)

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def plan_callback(self, msg):
        if len(msg.poses) > 0:
            self.last_plan = msg

    def loop(self):
        if self.map_msg is None or self.odom_msg is None:
            self.get_logger().info("Waiting for /map and /odom...")
            return
        self.destroy_timer(self.timer)
        self.send_action_goal()

    def send_action_goal(self):
        self.get_logger().info("Waiting for action server '/navigate_to_pose'...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server '/navigate_to_pose' not available. Exiting.")
            sys.exit(1)
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f"Sending goal to ({self.goal_x}, {self.goal_y})...")
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by action server.")
            self.action_done = True
            self.action_success = False
            sys.exit(1)
            
        self.get_logger().info("Goal accepted, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status code 4 represents SUCCEEDED
        if status == 4:
            self.get_logger().info("Action SUCCEEDED!")
            self.action_success = True
        else:
            self.get_logger().warn(f"Action finished with status code: {status}")
            self.action_success = False
            
        self.action_done = True
        self.verify_fallback()

    def verify_fallback(self):
        if not self.action_success:
            self.get_logger().error("FAIL: NavigateToPose action did not succeed.")
            sys.exit(1)
            
        if self.odom_msg is None:
            self.get_logger().error("FAIL: Odometry was not received.")
            sys.exit(1)
            
        rx = self.odom_msg.pose.pose.position.x
        ry = self.odom_msg.pose.pose.position.y
        
        dist_to_original = math.hypot(rx - self.goal_x, ry - self.goal_y)
        self.get_logger().info(f"Rover stopped at final position: ({rx:.2f}, {ry:.2f})")
        self.get_logger().info(f"Distance to original goal ({self.goal_x}, {self.goal_y}): {dist_to_original:.3f} m")
        
        # 1. Final position should NOT be exactly at the original goal
        # (original goal is at center of 5x5 box, so it should be at least 0.15m away)
        if dist_to_original < 0.15:
            self.get_logger().error("FAIL: Rover reached the original goal which was supposed to be unreachable inside the box!")
            sys.exit(1)
            
        # 2. Final position should be within 5.0m threshold limit
        if dist_to_original > 5.0:
            self.get_logger().error(f"FAIL: Rover stopped outside the maximum 5.0m fallback threshold! Distance: {dist_to_original:.3f} m")
            sys.exit(1)
            
        # 3. Final position should match the final plan end pose
        if self.last_plan is not None:
            plan_end_x = self.last_plan.poses[-1].pose.position.x
            plan_end_y = self.last_plan.poses[-1].pose.position.y
            dist_to_plan_end = math.hypot(rx - plan_end_x, ry - plan_end_y)
            self.get_logger().info(f"Distance to final planned end: {dist_to_plan_end:.3f} m")
            if dist_to_plan_end > 0.15:
                self.get_logger().error(f"FAIL: Rover final position is too far from planned destination! Distance: {dist_to_plan_end:.3f} m")
                sys.exit(1)
        
        self.get_logger().info(f"PASS: Lidar fallback verified. Rover safely stopped at fallback pose ({rx:.2f}, {ry:.2f}) at distance {dist_to_original:.2f} m from unreachable goal.")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TestHarnessFallback()
    exit_code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        exit_code = e.code
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
