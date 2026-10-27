#!/usr/bin/env python3

# pylint: disable=all
# mypy: ignore-errors

import math
import rclpy
from rclpy.node import Node
#from turtlebot3_msgs.msg import wp_list  # Import your custom message
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class path_publisher(Node):
    def __init__(self):
        super().__init__('path_test_publisher')
        
        # Defining parameters
        self.DS = 0.05
        self.R_turn = 0.5  # Sharp turn radius
        self.R_circle = 1.5 # Main circle radius

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Path, '/path', self.qos)
        self.timer = self.create_timer(1.0, self.publish_path)

        self.msg = Path()
        self.msg.header.frame_id = "odom"
        self.generate_track_message()
        
    def generate_track_message(self):
        # Create a path with: Straight -> U-turn -> Circle
        x_segments = []
        y_segments = []

        def append_segment(xs, ys):
            x_segments.append(xs)
            y_segments.append(ys)

        # 1) Straight line forward (0,0) -> (4,0)
        x1 = np.linspace(0.0, 4.0, int(4.0 / self.DS) + 1)
        y1 = np.zeros_like(x1)
        append_segment(x1, y1)

        # 2) U-turn (180 degrees)
        # Semicircle center at (4.0, 1.0), radius 1.0
        # From -pi/2 to pi/2
        R_uturn = 1.0
        theta2 = np.linspace(-math.pi / 2, math.pi / 2, int(math.pi * R_uturn / self.DS) + 1)
        x2 = 4.0 + R_uturn * np.cos(theta2)
        y2 = 1.0 + R_uturn * np.sin(theta2)
        append_segment(x2, y2)

        # 3) Straight back part of U-turn (4,2) -> (2,2)
        x3 = np.linspace(4.0, 2.0, int(2.0 / self.DS) + 1)
        y3 = np.full_like(x3, 2.0)
        append_segment(x3, y3)

        # 4) Circular path
        # Circle center at (0.5, 2.0), radius 1.5
        # Starts at (2,2), goes full circle
        R_circle = 1.5
        cx, cy = 0.5, 2.0
        theta4 = np.linspace(0.0, 2 * math.pi, int(2 * math.pi * R_circle / self.DS) + 1)
        x4 = cx + R_circle * np.cos(theta4)
        y4 = cy + R_circle * np.sin(theta4)
        append_segment(x4, y4)

        self.x_way_points_list = np.concatenate(x_segments)
        self.y_way_points_list = np.concatenate(y_segments)

        # Save to CSV for APF to load
        import pandas as pd
        import os
        ws_root = os.environ.get("ROAR_WS", str(Path(__file__).resolve().parents[5]))
        results_dir = os.path.join(ws_root, "reports", "pursuit")
        os.makedirs(results_dir, exist_ok=True)
        df = pd.DataFrame({
            'real_x': self.x_way_points_list,
            'real_y': self.y_way_points_list
        })
        csv_path = os.path.join(results_dir, "complex_path.csv")
        df.to_csv(csv_path, index=False)
        self.get_logger().info(f"Saved modified path (U-turn + Circle) to {csv_path}")

        self.msg.poses = []
        for x, y in zip(self.x_way_points_list, self.y_way_points_list):
            point = PoseStamped()
            point.pose.position.x = float(x)
            point.pose.position.y = float(y)
            self.msg.poses.append(point)

        self.get_logger().info(f"Generated U-turn path with {len(self.x_way_points_list)} waypoints.")    

    def publish_path(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.msg)
        self.get_logger().info("Published path message")

def main(args=None):
    rclpy.init(args=args)
    
    node = path_publisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()