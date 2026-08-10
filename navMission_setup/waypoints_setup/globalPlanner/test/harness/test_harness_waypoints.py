#!/usr/bin/env python3
import os
import sys
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, DurabilityPolicy

# Sequence of waypoints to visit on the 20m x 20m complex map
WAYPOINTS = [
    (18.0, 3.0),   # Waypoint 1: Bottom Right corner (navigates through the Wall 1 opening at x > 14.0m)
    (18.0, 18.0),  # Waypoint 2: Top Right corner
    (2.0, 18.0),   # Waypoint 3: Top Left corner (navigates through the Wall 2 opening at x < 6.0m)
    (2.0, 2.0)     # Waypoint 4: Return to Bottom Left
]

class TestHarnessWaypoints(Node):
    def __init__(self):
        super().__init__('test_harness_waypoints')
        
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('timeout_sec', 240.0)
        
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        self.get_logger().info("Multi-Waypoint Test Harness started.")
        self.get_logger().info(f"Start Position: ({self.start_x}, {self.start_y})")
        self.get_logger().info(f"Waypoints to visit: {WAYPOINTS}")
        
        # State
        self.map_msg = None
        self.odom_msg = None
        self.current_wp_idx = 0
        self.start_time = None
        self.waypoint_reach_times = []
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.5, self.loop)

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def loop(self):
        if self.map_msg is None or self.odom_msg is None:
            self.get_logger().info("Waiting for /map and /odom...")
            return
        self.destroy_timer(self.timer)
        self.get_logger().info("Map and Odom received. Delaying 1.5s to ensure planner registers them...")
        import time
        time.sleep(1.5)
        self.start_time = time.time()
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_wp_idx >= len(WAYPOINTS):
            self.report_success()
            return
            
        wx, wy = WAYPOINTS[self.current_wp_idx]
        self.get_logger().info(f"\n---> [Waypoint {self.current_wp_idx + 1}/{len(WAYPOINTS)}] Sending goal to ({wx:.2f}, {wy:.2f})...")
        
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server '/navigate_to_pose' not available. Exiting.")
            sys.exit(1)
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wx
        goal_msg.pose.pose.position.y = wy
        goal_msg.pose.pose.orientation.w = 1.0
        
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"FAIL: Waypoint {self.current_wp_idx + 1} was rejected by action server.")
            sys.exit(1)
            
        self.get_logger().info(f"Waypoint {self.current_wp_idx + 1} accepted, navigating...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status code 4 represents SUCCEEDED
        if status == 4:
            elapsed = time.time() - self.start_time
            self.waypoint_reach_times.append(elapsed)
            self.get_logger().info(f"PASS: Waypoint {self.current_wp_idx + 1} reached successfully in {elapsed:.2f}s!")
            
            # Verify final position matches waypoint within tolerance
            if self.odom_msg is not None:
                rx = self.odom_msg.pose.pose.position.x
                ry = self.odom_msg.pose.pose.position.y
                wx, wy = WAYPOINTS[self.current_wp_idx]
                dist = math.hypot(rx - wx, ry - wy)
                self.get_logger().info(f"Rover reached pose: ({rx:.2f}, {ry:.2f}) [Distance to target: {dist:.3f}m]")
                if dist > 0.35:
                    self.get_logger().error(f"FAIL: Rover stopped too far from waypoint {self.current_wp_idx + 1}! Dist: {dist:.3f}m")
                    sys.exit(1)
            
            # Advance to next waypoint
            self.current_wp_idx += 1
            self.send_next_waypoint()
        else:
            self.get_logger().error(f"FAIL: Failed to reach waypoint {self.current_wp_idx + 1}. Status code: {status}")
            sys.exit(1)

    def report_success(self):
        total_time = time.time() - self.start_time
        self.get_logger().info("\n==================================================")
        self.get_logger().info("             ALL WAYPOINTS VISITED!               ")
        self.get_logger().info("==================================================")
        for idx, t in enumerate(self.waypoint_reach_times):
            self.get_logger().info(f"Waypoint {idx + 1} {WAYPOINTS[idx]}: Reached at {t:.2f}s")
        self.get_logger().info(f"Total Navigation Duration: {total_time:.2f}s")
        self.get_logger().info("==================================================")
        
        # Save a summary report file
        results_dir = "/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/results"
        os.makedirs(results_dir, exist_ok=True)
        report_path = os.path.join(results_dir, "multi_waypoint_report.md")
        with open(report_path, 'w') as f:
            f.write("# Multi-Waypoint Navigation Performance Report\n\n")
            f.write("## Test Setup\n")
            f.write("- Map: 400x400 cells (20.0m x 20.0m complex canyon environment)\n")
            f.write(f"- Start position: ({self.start_x}, {self.start_y})\n")
            f.write("- Waypoints sequence:\n")
            for idx, wp in enumerate(WAYPOINTS):
                f.write(f"  {idx + 1}. Waypoint {idx + 1} at {wp}\n")
            f.write(f"\n## Performance Metrics\n")
            f.write(f"- Success: Yes (All {len(WAYPOINTS)} waypoints reached)\n")
            f.write(f"- Total Navigation Time: {total_time:.2f} seconds\n\n")
            f.write("| Waypoint | Coordinates | Reach Time (s) |\n")
            f.write("|---|---|---|\n")
            for idx, t in enumerate(self.waypoint_reach_times):
                f.write(f"| {idx + 1} | {WAYPOINTS[idx]} | {t:.2f} |\n")
                
        self.get_logger().info(f"Saved multi-waypoint report to: {report_path}")
        self.get_logger().info("PASS: Multi-Waypoint test completed successfully.")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TestHarnessWaypoints()
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
