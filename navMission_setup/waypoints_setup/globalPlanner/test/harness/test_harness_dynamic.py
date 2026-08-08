#!/usr/bin/env python3
import os
import csv
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

class TestHarnessDynamic(Node):
    def __init__(self):
        super().__init__('test_harness_dynamic')
        
        # Declare parameters
        self.declare_parameter('scenario', 'surprise_wall')
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('timeout_sec', 60.0)
        
        self.scenario = self.get_parameter('scenario').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        self.get_logger().info(f"Test Harness Dynamic starting scenario: '{self.scenario}'...")
        self.get_logger().info(f"Start: ({self.start_x}, {self.start_y}) -> Goal: ({self.goal_x}, {self.goal_y})")
        
        # State
        self.map_msg = None
        self.odom_msg = None
        self.plans_saved = []  # list of list of (x, y) coordinates
        self.version_count = 0
        self.action_done = False
        self.action_success = False
        self.plans_to_plot = []  # Defer plotting to avoid blocking callbacks
        
        # Subscriptions
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/global_plan', self.plan_callback, 10)
        
        # Action Client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Start execution loop
        self.timer = self.create_timer(0.5, self.loop)

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

    def plan_callback(self, msg):
        if len(msg.poses) == 0:
            return
            
        # Extract coordinates
        coords = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # Check if this plan is different from the last saved plan
        if len(self.plans_saved) > 0:
            last_coords = self.plans_saved[-1]
            if len(coords) == len(last_coords):
                # Check if coordinates match closely
                match = True
                for c1, c2 in zip(coords, last_coords):
                    if math.hypot(c1[0] - c2[0], c1[1] - c2[1]) > 0.01:
                        match = False
                        break
                if match:
                    # Duplicate plan, skip saving
                    return
        
        # New distinct plan!
        self.plans_saved.append(coords)
        self.version_count += 1
        self.get_logger().info(f"Captured plan version {self.version_count} with {len(coords)} poses.")
        self.validate_and_save_plan(msg, self.version_count)

    def validate_and_save_plan(self, plan_msg, version):
        if self.map_msg is None:
            self.get_logger().warn("Cannot validate plan: map not received yet.")
            return
            
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        map_data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))
        
        path_poses = plan_msg.poses
        
        # Check collision & connectivity
        for i, pose_stamped in enumerate(path_poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            
            if col < 0 or col >= width or row < 0 or row >= height:
                self.get_logger().error(f"FAIL: Pose at index {i} ({x:.2f}, {y:.2f}) is outside map boundaries!")
                self.save_single_plot(plan_msg, self.map_msg, version)
                sys.exit(1)
                
            cell_cost = map_data[row, col]
            # Lethal is 100.
            if cell_cost == 100:
                self.get_logger().error(f"FAIL: Pose at index {i} ({x:.2f}, {y:.2f}) collides with a lethal obstacle! Grid: [{row}, {col}]")
                self.save_single_plot(plan_msg, self.map_msg, version)
                sys.exit(1)
                
            # Check connectivity with next point
            if i < len(path_poses) - 1:
                next_x = path_poses[i+1].pose.position.x
                next_y = path_poses[i+1].pose.position.y
                step_dist = math.hypot(next_x - x, next_y - y)
                if i == 0 or i == len(path_poses) - 2:
                    max_step = 2.15 * resolution
                else:
                    max_step = math.sqrt(2.0) * resolution + 1e-4
                if step_dist > max_step:
                    self.get_logger().error(f"FAIL: Disconnected step from index {i} to {i+1}. Distance: {step_dist:.4f} m (Max expected: {max_step:.4f} m)")
                    self.save_single_plot(plan_msg, self.map_msg, version)
                    sys.exit(1)
                    
        # Create output directory
        results_dir = "/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/results"
        os.makedirs(results_dir, exist_ok=True)
        
        # Save CSV
        csv_path = os.path.join(results_dir, f"{self.scenario}_path_v{version}.csv")
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for pose_stamped in path_poses:
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
        
        # Defer plotting
        rx = self.odom_msg.pose.pose.position.x if self.odom_msg is not None else self.start_x
        ry = self.odom_msg.pose.pose.position.y if self.odom_msg is not None else self.start_y
        self.plans_to_plot.append((version, plan_msg, self.map_msg, rx, ry))

    def save_single_plot(self, plan_msg, map_msg, version, rx=None, ry=None):
        try:
            width = map_msg.info.width
            height = map_msg.info.height
            resolution = map_msg.info.resolution
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y
            map_data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))
            
            results_dir = "/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/results"
            plot_path = os.path.join(results_dir, f"{self.scenario}_plot_v{version}.png")
            
            plt.figure(figsize=(10, 10))
            plt.imshow(map_data, cmap='viridis', origin='lower', extent=[origin_x, origin_x + width*resolution, origin_y, origin_y + height*resolution])
            
            px = [p.pose.position.x for p in plan_msg.poses]
            py = [p.pose.position.y for p in plan_msg.poses]
            plt.plot(px, py, 'r-', linewidth=3, label=f'Planned Path (v{version})')
            
            if rx is not None and ry is not None:
                plt.scatter([rx], [ry], color='red', marker='o', s=100, zorder=6, label='Robot Position')
                
            plt.scatter([self.start_x], [self.start_y], color='blue', s=100, zorder=5, label='Start')
            plt.scatter([self.goal_x], [self.goal_y], color='gold', marker='*', s=200, zorder=5, label='Goal')
            
            plt.title(f"D* Lite Path Update — Scenario: {self.scenario} (v{version})")
            plt.xlabel("X (meters)")
            plt.ylabel("Y (meters)")
            plt.legend()
            plt.savefig(plot_path, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Saved plot image to: {plot_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving plot: {e}")

    def loop(self):
        # Wait for map to be available
        if self.map_msg is None:
            self.get_logger().info("Waiting for /map...")
            return
            
        # Cancel loop timer and send goal
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
            return
            
        self.get_logger().info("Goal accepted by action server, waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # Status code 4 represents SUCCEEDED
        if status == 4:
            self.get_logger().info("Goal SUCCEEDED!")
            self.action_success = True
        else:
            self.get_logger().warn(f"Goal finished with status code: {status}")
            self.action_success = False
            
        self.action_done = True
        self.verify_completion()

    def verify_completion(self):
        if not self.action_success:
            self.get_logger().error("FAIL: Goal did not succeed.")
            sys.exit(1)
            
        if self.version_count < 2 and self.scenario in ['known_maze_blocked', 'lidar_maze_120']:
            self.get_logger().error(f"FAIL: Expected dynamic replanning versions, but only got {self.version_count} plan version(s).")
            sys.exit(1)
            
        # Plot representative deferred plans now (first, middle, last)
        self.get_logger().info(f"Generating key path plots from {len(self.plans_to_plot)} total plans...")
        if len(self.plans_to_plot) > 0:
            indices = {0, len(self.plans_to_plot) - 1}
            if len(self.plans_to_plot) > 2:
                indices.add(len(self.plans_to_plot) // 2)
            for idx in sorted(indices):
                version, plan_msg, map_msg, rx, ry = self.plans_to_plot[idx]
                self.save_single_plot(plan_msg, map_msg, version, rx, ry)
            
        self.get_logger().info(f"PASS: Dynamic scenario '{self.scenario}' completed successfully with {self.version_count} distinct paths planned.")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TestHarnessDynamic()
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
