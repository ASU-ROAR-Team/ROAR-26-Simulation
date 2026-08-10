#!/usr/bin/env python3
"""Phase 3 Hybrid Flat Terrain Test Harness.

Extends the dynamic harness with:
- corridor_block: verifies corridor rerouting when blocked mid-path
- goal_change_dynamic: verifies planner handles a new goal issued mid-navigation
"""
import os
import csv
import sys
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy


class TestHarnessHybrid(Node):
    def __init__(self):
        super().__init__('test_harness_hybrid')

        # Declare parameters
        self.declare_parameter('scenario', 'corridor_block')
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 4.5)
        self.declare_parameter('new_goal_x', 4.5)
        self.declare_parameter('new_goal_y', 0.5)
        self.declare_parameter('timeout_sec', 90.0)

        self.scenario = self.get_parameter('scenario').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.new_goal_x = self.get_parameter('new_goal_x').value
        self.new_goal_y = self.get_parameter('new_goal_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.get_logger().info(f"Hybrid Test Harness starting scenario: '{self.scenario}'")
        self.get_logger().info(f"Start: ({self.start_x}, {self.start_y}) -> Goal: ({self.goal_x}, {self.goal_y})")

        # State
        self.map_msg = None
        self.odom_msg = None
        self.plans_saved = []
        self.version_count = 0
        self.action_done = False
        self.action_success = False
        self.plans_to_plot = []

        # Goal-change tracking
        self.second_goal_sent = False
        self.first_goal_handle = None
        self.second_goal_handle = None
        self.final_goal_x = self.goal_x
        self.final_goal_y = self.goal_y

        # Subscriptions
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.plan_sub = self.create_subscription(Path, '/global_plan', self.plan_callback, 10)

        # Action client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Start loop
        self.timer = self.create_timer(0.5, self.loop)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def map_callback(self, msg):
        self.map_msg = msg

    def odom_callback(self, msg):
        self.odom_msg = msg

        # For goal_change_dynamic: send a new goal once robot is halfway
        if self.scenario == 'goal_change_dynamic' and not self.second_goal_sent and self.action_done is False:
            rx = msg.pose.pose.position.x
            ry = msg.pose.pose.position.y
            dist_from_start = math.hypot(rx - self.start_x, ry - self.start_y)
            if dist_from_start >= 1.5:
                self.get_logger().info("Sending NEW goal mid-navigation (goal_change_dynamic)!")
                self.second_goal_sent = True
                self.final_goal_x = self.new_goal_x
                self.final_goal_y = self.new_goal_y
                self.send_action_goal(self.new_goal_x, self.new_goal_y)

    def plan_callback(self, msg):
        if len(msg.poses) == 0:
            return

        coords = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

        # Deduplicate
        if len(self.plans_saved) > 0:
            last = self.plans_saved[-1]
            if len(coords) == len(last):
                match = all(
                    math.hypot(c1[0] - c2[0], c1[1] - c2[1]) <= 0.01
                    for c1, c2 in zip(coords, last)
                )
                if match:
                    return

        self.plans_saved.append(coords)
        self.version_count += 1
        self.get_logger().info(f"Captured plan version {self.version_count} with {len(coords)} poses.")
        self.validate_and_save_plan(msg, self.version_count)

    # ── Validation & Plotting ──────────────────────────────────────────────────

    def validate_and_save_plan(self, plan_msg, version):
        if self.map_msg is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        map_data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))

        for i, ps in enumerate(plan_msg.poses):
            x = ps.pose.position.x
            y = ps.pose.position.y
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)

            if col < 0 or col >= width or row < 0 or row >= height:
                self.get_logger().error(f"FAIL: Pose {i} ({x:.2f},{y:.2f}) outside map!")
                self.save_single_plot(plan_msg, self.map_msg, version)
                sys.exit(1)

            if map_data[row, col] == 100:
                self.get_logger().error(f"FAIL: Pose {i} ({x:.2f},{y:.2f}) on lethal obstacle!")
                self.save_single_plot(plan_msg, self.map_msg, version)
                sys.exit(1)

            if i < len(plan_msg.poses) - 1:
                nx = plan_msg.poses[i + 1].pose.position.x
                ny = plan_msg.poses[i + 1].pose.position.y
                step = math.hypot(nx - x, ny - y)
                max_step = 2.15 * resolution if (i == 0 or i == len(plan_msg.poses) - 2) else math.sqrt(2.0) * resolution + 1e-4
                if step > max_step:
                    self.get_logger().error(f"FAIL: Disconnected step {i}->{i+1}: {step:.4f}m > {max_step:.4f}m")
                    self.save_single_plot(plan_msg, self.map_msg, version)
                    sys.exit(1)

        # Save CSV
        results_dir = "/home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/results"
        os.makedirs(results_dir, exist_ok=True)
        csv_path = os.path.join(results_dir, f"{self.scenario}_path_v{version}.csv")
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for ps in plan_msg.poses:
                writer.writerow([ps.pose.position.x, ps.pose.position.y])

        rx = self.odom_msg.pose.pose.position.x if self.odom_msg else self.start_x
        ry = self.odom_msg.pose.pose.position.y if self.odom_msg else self.start_y
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
            plt.imshow(map_data, cmap='viridis', origin='lower',
                       extent=[origin_x, origin_x + width * resolution,
                                origin_y, origin_y + height * resolution])

            px = [p.pose.position.x for p in plan_msg.poses]
            py = [p.pose.position.y for p in plan_msg.poses]
            plt.plot(px, py, 'r-', linewidth=3, label=f'Plan v{version}')

            if rx is not None:
                plt.scatter([rx], [ry], color='red', s=100, zorder=6, label='Robot')

            plt.scatter([self.start_x], [self.start_y], color='blue', s=100, zorder=5, label='Start')
            plt.scatter([self.final_goal_x], [self.final_goal_y], color='gold', marker='*', s=200, zorder=5, label='Goal')

            plt.title(f"D* Lite Hybrid — {self.scenario} (v{version})")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.legend()
            plt.savefig(plot_path, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Saved plot: {plot_path}")
        except Exception as e:
            self.get_logger().error(f"Plot error: {e}")

    # ── Action Server Interaction ───────────────────────────────────────────────

    def loop(self):
        if self.map_msg is None:
            self.get_logger().info("Waiting for /map...")
            return
        self.destroy_timer(self.timer)
        self.send_action_goal(self.goal_x, self.goal_y)

    def send_action_goal(self, goal_x, goal_y):
        self.get_logger().info(f"Waiting for action server...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            sys.exit(1)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to ({goal_x:.2f}, {goal_y:.2f})...")
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            sys.exit(1)

        if self.scenario == 'goal_change_dynamic':
            if not self.second_goal_sent:
                self.first_goal_handle = goal_handle
                self.get_logger().info("First goal accepted.")
            else:
                self.second_goal_handle = goal_handle
                self.get_logger().info("Second goal accepted.")
        else:
            self.first_goal_handle = goal_handle
            self.get_logger().info("Goal accepted, waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.get_result_callback(f, goal_handle))

    def get_result_callback(self, future, goal_handle):
        status = future.result().status

        # If this is the preempted first goal of a dynamic goal change, we expect it to be aborted/canceled.
        # We should NOT count it as overall failure or success — just log and wait for the second goal.
        if self.scenario == 'goal_change_dynamic':
            if goal_handle == self.first_goal_handle:
                self.get_logger().info(f"First goal finished/preempted as expected with status: {status}")
                return
            elif goal_handle == self.second_goal_handle:
                self.action_success = (status == 4)
                if status == 4:
                    self.get_logger().info("Second goal SUCCEEDED!")
                else:
                    self.get_logger().warn(f"Second goal finished with status: {status}")
            else:
                # Fallback if handle mapping is unclear
                self.action_success = (status == 4)
        else:
            self.action_success = (status == 4)
            if status == 4:
                self.get_logger().info("Goal SUCCEEDED!")
            else:
                self.get_logger().warn(f"Goal finished with status: {status}")

        self.action_done = True
        self.verify_completion()

    def verify_completion(self):
        if not self.action_success:
            self.get_logger().error("FAIL: Goal did not succeed.")
            sys.exit(1)

        if self.version_count < 2:
            self.get_logger().error(f"FAIL: Expected replanning but got only {self.version_count} plan(s).")
            sys.exit(1)

        # Plot representative plans
        self.get_logger().info(f"Generating plots from {len(self.plans_to_plot)} total plans...")
        if self.plans_to_plot:
            indices = {0, len(self.plans_to_plot) - 1}
            if len(self.plans_to_plot) > 2:
                indices.add(len(self.plans_to_plot) // 2)
            for idx in sorted(indices):
                version, plan_msg, map_msg, rx, ry = self.plans_to_plot[idx]
                self.save_single_plot(plan_msg, map_msg, version, rx, ry)

        self.get_logger().info(f"PASS: Hybrid scenario '{self.scenario}' completed with {self.version_count} distinct plans.")
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TestHarnessHybrid()
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
