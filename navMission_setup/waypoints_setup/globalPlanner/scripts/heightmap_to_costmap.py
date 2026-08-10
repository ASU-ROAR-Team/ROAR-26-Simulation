#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import cv2
import math


class HeightmapToCostmap(Node):
    def __init__(self):
        super().__init__('heightmap_to_costmap')
        # parameters
        self.declare_parameter('heightmap_topic', '/active_map/heightmap')
        self.declare_parameter('merged_costmap_topic', '/terrain_costmap')
        self.declare_parameter('height_factor', 100.0)
        self.declare_parameter('MAX_SAFE_SLOPE', 0.5)
        self.declare_parameter('MAX_SAFE_ROUGHNESS', 0.5)
        self.declare_parameter('steepness_weight', 0.5)
        self.declare_parameter('roughness_weight', 0.5)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscribers
        self.heightmap_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('heightmap_topic').value,
            self.heightmap_callback,
            qos)

        # Publisher (single merged costmap)
        self.merged_costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('merged_costmap_topic').value,
            qos)

        self.get_logger().info("Heightmap to Costmap Converter Ready.")

    def heightmap_callback(self, msg):
        start_time = time.time()
        width = msg.info.width
        height = msg.info.height
        raw_heightmap = msg.data
        heightmap = np.array(raw_heightmap, dtype=np.int8).reshape((height, width))

        unknown_mask = (heightmap == -1)
        kernel = np.ones((3, 3), np.uint8)
        dilated_mask = cv2.dilate(unknown_mask.astype(np.uint8), kernel, iterations=2).astype(bool)

        height_factor = self.get_parameter('height_factor').value
        heightmap = (heightmap / height_factor).astype(np.float32)
        heightmap = cv2.GaussianBlur(heightmap, (3, 3), 0)

        # Compute steepness costmap
        sobel_x = cv2.Sobel(heightmap, cv2.CV_64F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(heightmap, cv2.CV_64F, 0, 1, ksize=3)
        steepness_costmap = np.sqrt(sobel_x ** 2 + sobel_y ** 2)

        # Compute roughness costmap
        laplacian = cv2.Laplacian(heightmap, cv2.CV_64F)
        roughness_costmap = np.abs(laplacian)

        # Normalize each to 0-100
        steepness_costmap = np.clip(
            (steepness_costmap / self.get_parameter('MAX_SAFE_SLOPE').value) * 100.0, 0, 100)
        roughness_costmap = np.clip(
            (roughness_costmap / self.get_parameter('MAX_SAFE_ROUGHNESS').value) * 100.0, 0, 100)

        # Merge with configurable weights
        steepness_weight = self.get_parameter('steepness_weight').value
        roughness_weight = self.get_parameter('roughness_weight').value
        merged_costmap = np.clip(
            steepness_costmap * steepness_weight + roughness_costmap * roughness_weight,
            0, 100)

        merged_costmap[dilated_mask] = -1

        # Convert to int8
        merged_costmap = merged_costmap.astype(np.int8)

        # Publish merged costmap
        merged_msg = OccupancyGrid()
        merged_msg.header = msg.header
        merged_msg.info = msg.info
        merged_msg.data = merged_costmap.flatten().tolist()
        self.merged_costmap_pub.publish(merged_msg)

        end_time = time.time()
        self.get_logger().info(f"Processed heightmap to merged costmap in {end_time - start_time:.2f} seconds.")


def main(args=None):
    rclpy.init(args=args)
    node = HeightmapToCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
