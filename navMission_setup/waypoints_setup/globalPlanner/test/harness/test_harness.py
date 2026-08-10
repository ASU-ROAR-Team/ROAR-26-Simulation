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
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

class TestHarness(Node):
    def __init__(self):
        super().__init__('test_harness')
        
        # Declare parameters
        self.declare_parameter('scenario', 'wall')
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('timeout_sec', 30.0)
        self.declare_parameter('expect_fail', False)
        
        self.scenario = self.get_parameter('scenario').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.expect_fail = self.get_parameter('expect_fail').value
        
        self.get_logger().info(f"Test Harness starting scenario: '{self.scenario}'...")
        self.get_logger().info(f"Goal: ({self.goal_x}, {self.goal_y}), Expecting failure: {self.expect_fail}")
        
        # State
        self.map_msg = None
        self.plan_msg = None
        self.first_plan_msg = None  # To save the complete initial path before robot starts moving
        self.action_done = False
        self.action_success = False
        
        # Subscriptions
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.plan_sub = self.create_subscription(Path, '/global_plan', self.plan_callback, 10)
        
        # Action Client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Start execution loop
        self.timer = self.create_timer(0.5, self.loop)

    def map_callback(self, msg):
        self.map_msg = msg

    def plan_callback(self, msg):
        self.plan_msg = msg
        if self.first_plan_msg is None:
            self.first_plan_msg = msg
            self.get_logger().info(f"Captured initial plan with {len(msg.poses)} poses.")
        else:
            self.get_logger().info(f"Captured plan update with {len(msg.poses)} poses.")

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
        # Status code 4 represents SUCCEEDED in action_msgs/msg/GoalStatus
        if status == 4:
            self.get_logger().info("Goal SUCCEEDED!")
            self.action_success = True
        else:
            self.get_logger().warn(f"Goal finished with status code: {status}")
            self.action_success = False
            
        self.action_done = True
        self.run_checks_and_save()

    def run_checks_and_save(self):
        # Validate test outcome against expectation
        if self.expect_fail:
            if not self.action_success:
                self.get_logger().info("PASS: Planning correctly failed or aborted for unreachable goal.")
                sys.exit(0)
            else:
                self.get_logger().error("FAIL: Expected planning to fail, but it succeeded.")
                sys.exit(1)
                
        if not self.action_success:
            self.get_logger().error("FAIL: Goal did not succeed.")
            sys.exit(1)
            
        if self.first_plan_msg is None or len(self.first_plan_msg.poses) == 0:
            self.get_logger().error("FAIL: No global plan was received.")
            sys.exit(1)
            
        # Parse map parameters
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        map_data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))
        
        # Verify path correctness
        path_poses = self.first_plan_msg.poses
        self.get_logger().info(f"Running validation checks on {len(path_poses)} poses...")
        
        # Check start & goal poses
        start_pose = path_poses[0].pose.position
        end_pose = path_poses[-1].pose.position
        
        start_dist = math.hypot(start_pose.x - self.start_x, start_pose.y - self.start_y)
        end_dist = math.hypot(end_pose.x - self.goal_x, end_pose.y - self.goal_y)
        
        # Tolerance: must be within 2 grid cells
        tol = resolution * 2.0
        
        if start_dist > tol:
            self.get_logger().error(f"FAIL: Path start ({start_pose.x:.2f}, {start_pose.y:.2f}) does not match expected ({self.start_x}, {self.start_y}). Dist: {start_dist:.2f}")
            sys.exit(1)
            
        if end_dist > tol:
            self.get_logger().error(f"FAIL: Path end ({end_pose.x:.2f}, {end_pose.y:.2f}) does not match expected goal ({self.goal_x}, {self.goal_y}). Dist: {end_dist:.2f}")
            sys.exit(1)
            
        # Check for collisions & connectivity
        for i, pose_stamped in enumerate(path_poses):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            
            # Convert to grid index
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            
            if col < 0 or col >= width or row < 0 or row >= height:
                self.get_logger().error(f"FAIL: Pose at index {i} ({x:.2f}, {y:.2f}) is outside map boundaries!")
                sys.exit(1)
                
            cell_cost = map_data[row, col]
            # Lethal is 100
            if cell_cost == 100:
                self.get_logger().error(f"FAIL: Pose at index {i} ({x:.2f}, {y:.2f}) collides with a lethal obstacle! Grid: [{row}, {col}]")
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
                    sys.exit(1)
                    
        self.get_logger().info("PASS: All validation checks (start/end match, collision-free, connectivity) PASSED!")
        
        # Save results
        results_dir = '/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/results'
        os.makedirs(results_dir, exist_ok=True)
        
        csv_path = os.path.join(results_dir, f"{self.scenario}_path.csv")
        with open(csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['x', 'y'])
            for pose_stamped in path_poses:
                writer.writerow([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
        self.get_logger().info(f"Saved path coordinates to: {csv_path}")
        
        # Matplotlib Plot
        plot_path = os.path.join(results_dir, f"{self.scenario}_plot.png")
        plt.figure(figsize=(10, 10))
        
        # Reshape and plot costmap
        # Matplotlib origin='lower' puts (0,0) at bottom-left, which matches ROS OccupancyGrid frame
        plt.imshow(map_data, cmap='viridis', origin='lower', extent=[origin_x, origin_x + width*resolution, origin_y, origin_y + height*resolution])
        
        # Plot path
        px = [p.pose.position.x for p in path_poses]
        py = [p.pose.position.y for p in path_poses]
        plt.plot(px, py, 'r-', linewidth=3, label='Planned Path')
        
        # Plot start and goal
        plt.scatter([self.start_x], [self.start_y], color='blue', s=100, zorder=5, label='Start')
        plt.scatter([self.goal_x], [self.goal_y], color='gold', marker='*', s=200, zorder=5, label='Goal')
        
        plt.title(f"D* Lite Path — Scenario: {self.scenario}")
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.legend()
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.savefig(plot_path, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f"Saved plot image to: {plot_path}")
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TestHarness()
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
