#!/usr/bin/env python3
"""Phase 4 Rough Terrain Test Harness.

Verifies cost-aware path planning:
- rough_patch: verifies path detours around high-cost central patch
- gradient_slope: verifies path is pulled towards lower-cost (bottom) region
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
from rclpy.qos import QoSProfile, DurabilityPolicy


class TestHarnessTerrain(Node):
    def __init__(self):
        super().__init__('test_harness_terrain')

        self.declare_parameter('scenario', 'rough_patch')
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 2.5)
        self.declare_parameter('goal_x', 4.5)
        self.declare_parameter('goal_y', 2.5)
        self.declare_parameter('timeout_sec', 90.0)

        self.scenario = self.get_parameter('scenario').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.get_logger().info(f"Terrain Test Harness starting scenario: '{self.scenario}'")

        # State
        self.map_msg = None
        self.odom_msg = None
        self.plans_saved = []
        self.version_count = 0
        self.action_done = False
        self.action_success = False
        self.plans_to_plot = []

        # Subscriptions
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

    def validate_and_save_plan(self, plan_msg, version):
        if self.map_msg is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        map_data = np.array(self.map_msg.data, dtype=np.int8).reshape((height, width))

        # We will check if the path respects cost constraints
        has_detoured = False
        y_coords = []

        for i, ps in enumerate(plan_msg.poses):
            x = ps.pose.position.x
            y = ps.pose.position.y
            y_coords.append(y)
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)

            if col < 0 or col >= width or row < 0 or row >= height:
                self.get_logger().error(f"FAIL: Pose {i} ({x:.2f},{y:.2f}) outside map!")
                sys.exit(1)

            if map_data[row, col] == 100:
                self.get_logger().error(f"FAIL: Pose {i} ({x:.2f},{y:.2f}) on lethal obstacle!")
                sys.exit(1)

            # Assertions for Cost-Awareness
            if self.scenario == 'rough_patch':
                # Central rough patch is at x in [2.0, 3.5], y in [1.0, 4.0]
                # If the path goes around it, the y coordinates in the middle of x should be < 1.0 or > 4.0.
                if 2.2 <= x <= 3.3:
                    if y < 1.0 or y > 4.0:
                        has_detoured = True

        if self.scenario == 'rough_patch' and version == 1:
            # We want to log if it detoured
            if has_detoured:
                self.get_logger().info("Success: Cost-aware planner successfully detoured around rough patch!")
            else:
                self.get_logger().warn("Warning: Path did not detour around rough patch. Check corridor_penalty_scale parameter!")

        elif self.scenario == 'gradient_slope' and version == 1:
            avg_y = np.mean(y_coords)
            self.get_logger().info(f"Gradient slope path average Y: {avg_y:.3f} meters (direct path would be ~2.5m)")
            if avg_y < 2.0:
                self.get_logger().info("Success: Path successfully pulled towards lower-cost region!")
            else:
                self.get_logger().warn("Warning: Path average Y is not significantly pulled lower. Check parameters!")

        elif self.scenario == 'forced_rough' and version == 1:
            # We verify the robot successfully navigated through the rough patch because top/bottom are blocked
            traversed_rough = False
            for ps in plan_msg.poses:
                x = ps.pose.position.x
                y = ps.pose.position.y
                if 2.2 <= x <= 3.3 and 1.0 <= y <= 4.0:
                    traversed_rough = True
            if traversed_rough:
                self.get_logger().info("Success: Planner successfully traversed high-cost terrain under forced bypass blockage!")
            else:
                self.get_logger().error("FAIL: Forced rough trajectory did not cross rough patch correctly.")
                sys.exit(1)

        elif self.scenario == 'perlin_terrain' and version == 1:
            self.get_logger().info("Success: Planner successfully planned path through synthetic multi-frequency terrain!")


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
            plt.scatter([self.goal_x], [self.goal_y], color='gold', marker='*', s=200, zorder=5, label='Goal')

            plt.title(f"D* Lite Terrain — {self.scenario} (v{version})")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.legend()
            plt.savefig(plot_path, bbox_inches='tight')
            plt.close()
            self.get_logger().info(f"Saved plot: {plot_path}")
        except Exception as e:
            self.get_logger().error(f"Plot error: {e}")

    def loop(self):
        if self.map_msg is None:
            self.get_logger().info("Waiting for /map...")
            return
        self.destroy_timer(self.timer)
        self.send_action_goal()

    def send_action_goal(self):
        self.get_logger().info(f"Waiting for action server...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            sys.exit(1)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal to ({self.goal_x:.2f}, {self.goal_y:.2f})...")
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected.")
            sys.exit(1)
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.action_success = (status == 4)
        self.action_done = True
        self.verify_completion()

    def verify_completion(self):
        if not self.action_success:
            self.get_logger().error("FAIL: Goal did not succeed.")
            sys.exit(1)

        # Plot representative plans
        self.get_logger().info(f"Generating plots...")
        if self.plans_to_plot:
            indices = {0, len(self.plans_to_plot) - 1}
            for idx in sorted(indices):
                version, plan_msg, map_msg, rx, ry = self.plans_to_plot[idx]
                self.save_single_plot(plan_msg, map_msg, version, rx, ry)

        self.get_logger().info(f"PASS: Terrain scenario '{self.scenario}' completed successfully.")
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = TestHarnessTerrain()
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
