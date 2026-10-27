#!/usr/bin/env python3
"""Unified Costmap Fuser — derives lethal obstacles and terrain costs purely
from the 2.5D heightmap, then applies an inflation layer around lethal cells.

The 2D SLAM /map is NOT used — it is itself derived from the heightmap, so
subscribing to it would be redundant.

Output encoding on /unified_costmap (OccupancyGrid):
  -1   = Unknown / unmapped
  0    = Perfectly flat, free space
  1-99 = Terrain difficulty OR inflation cost (higher = steeper/rougher/closer to obstacle)
  100  = Lethal (cliff, steep slope, or within inscribed radius of an obstacle)
"""
import rclpy
import numpy as np
import cv2
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray


class HeightmapToCostmap(Node):
    def __init__(self):
        super().__init__('heightmap_to_costmap')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('heightmap_topic', '/active_map/heightmap')
        self.declare_parameter('heightmap_range_topic', '/active_map/heightmap_range')
        self.declare_parameter('unified_costmap_topic', '/unified_costmap')
        self.declare_parameter('occupancy_heightmap_scale', 0.015)
        self.declare_parameter('MAX_SAFE_SLOPE', 0.5)
        self.declare_parameter('MAX_SAFE_ROUGHNESS', 0.5)
        self.declare_parameter('steepness_weight', 0.5)
        self.declare_parameter('roughness_weight', 0.5)
        self.declare_parameter('max_traversable_height_diff', 0.30)
        self.declare_parameter('publish_rate', 1.0)
        # Inflation layer parameters
        self.declare_parameter('inscribed_radius', 0.25)   # meters — cells within this of lethal → also lethal
        self.declare_parameter('inflation_radius', 0.8)    # meters — cells within this get decaying cost
        self.declare_parameter('inflation_cost_scaling', 3.0)  # exponential decay factor for inflation

        self.hm_topic = self.get_parameter('heightmap_topic').value
        self.hm_range_topic = self.get_parameter('heightmap_range_topic').value
        self.out_topic = self.get_parameter('unified_costmap_topic').value
        self.hm_scale = self.get_parameter('occupancy_heightmap_scale').value
        self.max_slope = self.get_parameter('MAX_SAFE_SLOPE').value
        self.max_roughness = self.get_parameter('MAX_SAFE_ROUGHNESS').value
        self.steep_w = self.get_parameter('steepness_weight').value
        self.rough_w = self.get_parameter('roughness_weight').value
        self.max_h_diff = self.get_parameter('max_traversable_height_diff').value
        self.pub_rate = self.get_parameter('publish_rate').value
        self.inscribed_radius = self.get_parameter('inscribed_radius').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.inflation_scaling = self.get_parameter('inflation_cost_scaling').value

        # State
        self.latest_hm = None
        self.hm_range = None
        self.last_dims = None
        self.first_publish = True
        self._cached_meshgrid = None

        # QoS — TRANSIENT_LOCAL so late-subscribing D* receives last message
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Subscriptions — heightmap only (no /map needed)
        self.create_subscription(OccupancyGrid, self.hm_topic, self.hm_cb, qos)
        self.create_subscription(Float32MultiArray, self.hm_range_topic, self.range_cb, qos)

        # Publisher
        self.pub = self.create_publisher(OccupancyGrid, self.out_topic, qos)

        # Timer-based publish at configured rate
        self.timer = self.create_timer(1.0 / self.pub_rate, self.timer_cb)
        self.get_logger().info(
            f"Unified Costmap Fuser ready (heightmap-only mode). "
            f"Subscribed to {self.hm_topic}, {self.hm_range_topic}. "
            f"Publishing on {self.out_topic} at {self.pub_rate} Hz. "
            f"Inflation: inscribed={self.inscribed_radius}m, "
            f"radius={self.inflation_radius}m, scaling={self.inflation_scaling}")

    def hm_cb(self, msg):
        self.latest_hm = msg

    def range_cb(self, msg):
        if len(msg.data) >= 2:
            self.hm_range = [msg.data[0], msg.data[1]]

    def timer_cb(self):
        if self.latest_hm is None:
            return

        start_time = time.time()
        hm_msg = self.latest_hm

        H_w = hm_msg.info.width
        H_h = hm_msg.info.height
        res = hm_msg.info.resolution

        if res <= 0:
            self.get_logger().warn("Invalid resolution in heightmap.")
            return

        # Validate data size
        if len(hm_msg.data) != H_w * H_h:
            self.get_logger().warn("Heightmap data size mismatch, skipping.")
            return

        # Log and cache when dimensions change
        if self.last_dims != (H_w, H_h):
            self.get_logger().info(
                f"Heightmap grid: {H_w}x{H_h}, res={res:.3f}m.")
            self.last_dims = (H_w, H_h)
            self._cached_meshgrid = np.meshgrid(np.arange(H_w), np.arange(H_h))

        # ─── Step 1: Decode heightmap to meters ─────────────────────────
        hm_data = np.array(hm_msg.data, dtype=np.float32).reshape((H_h, H_w))
        valid_hm = (hm_data != -1)
        h_meters = np.zeros_like(hm_data, dtype=np.float32)

        if self.hm_range is not None:
            h_min, h_range = self.hm_range
            h_meters[valid_hm] = h_min + (hm_data[valid_hm] / 100.0) * h_range
        else:
            h_meters[valid_hm] = hm_data[valid_hm] * self.hm_scale

        # Guard against all-invalid heightmap (NaN poisoning)
        if np.any(valid_hm):
            h_meters[~valid_hm] = np.mean(h_meters[valid_hm])
        else:
            out_msg = OccupancyGrid()
            out_msg.header = hm_msg.header
            out_msg.info = hm_msg.info
            out_msg.data = [-1] * (H_w * H_h)
            self.pub.publish(out_msg)
            return

        # ─── Step 2: Compute terrain derivatives ────────────────────────
        h_blur = cv2.GaussianBlur(h_meters, (3, 3), 0)

        # 1st derivative (steepness) via Sobel
        sobel_x = cv2.Sobel(h_blur, cv2.CV_32F, 1, 0, ksize=3)
        sobel_y = cv2.Sobel(h_blur, cv2.CV_32F, 0, 1, ksize=3)
        steepness = np.sqrt(sobel_x ** 2 + sobel_y ** 2)

        # 2nd derivative (roughness) via Laplacian
        roughness = np.abs(cv2.Laplacian(h_blur, cv2.CV_32F))

        # Normalize to 0-99 (100 is reserved for lethal)
        norm_steep = np.clip((steepness / self.max_slope) * 99.0, 0, 99)
        norm_rough = np.clip((roughness / self.max_roughness) * 99.0, 0, 99)

        # Merge with configurable weights
        terrain_cost = np.clip(
            norm_steep * self.steep_w + norm_rough * self.rough_w, 0, 99)

        # ─── Step 3: Detect lethal cells from heightmap ─────────────────
        # Steep slopes → lethal
        steep_lethal = steepness > self.max_slope

        # Height diffs (cliffs) → lethal (3x3 max-min filter)
        kernel_3x3 = np.ones((3, 3), np.uint8)
        max_h = cv2.dilate(h_blur, kernel_3x3)
        min_h = cv2.erode(h_blur, kernel_3x3)
        diff_lethal = (max_h - min_h) > self.max_h_diff

        lethal_mask = (steep_lethal | diff_lethal) & valid_hm

        # ─── Step 4: Inflation layer ────────────────────────────────────
        # Build binary lethal image for distance transform
        lethal_binary = lethal_mask.astype(np.uint8)  # 1 = lethal, 0 = free

        # Distance transform: compute distance from each cell to nearest lethal
        # cv2.distanceTransform needs inverted input (0 = obstacle)
        dist_map = cv2.distanceTransform(
            (1 - lethal_binary), cv2.DIST_L2, 5) * res  # convert to meters

        # Inscribed inflation: cells within inscribed_radius → also lethal
        inscribed_mask = dist_map < self.inscribed_radius

        # Gradual inflation: exponential decay between inscribed and inflation radius
        # inflation_cost = 99 * exp(-scaling * (dist - inscribed_radius) / (inflation_radius - inscribed_radius))
        inflation_cost = np.zeros_like(dist_map, dtype=np.float64)
        inflation_band = (dist_map >= self.inscribed_radius) & (dist_map < self.inflation_radius)
        if self.inflation_radius > self.inscribed_radius:
            normalized_dist = (dist_map[inflation_band] - self.inscribed_radius) / \
                              (self.inflation_radius - self.inscribed_radius)
            inflation_cost[inflation_band] = 99.0 * np.exp(-self.inflation_scaling * normalized_dist)

        # ─── Step 5: Combine everything ─────────────────────────────────
        # Start with terrain cost as base
        out = terrain_cost.copy()

        # Overlay inflation cost (take max of terrain and inflation)
        out = np.maximum(out, inflation_cost)

        # Mark lethal cells (original + inscribed inflation)
        all_lethal = lethal_mask | inscribed_mask
        out[all_lethal] = 100

        # Mark unknown cells
        hm_unknown = ~valid_hm
        out[hm_unknown] = -1

        # Clamp non-lethal, non-unknown to [0, 99]
        free_mask = ~all_lethal & valid_hm
        out[free_mask] = np.clip(out[free_mask], 0, 99)

        out = out.astype(np.int8)

        # ─── Step 6: Dilate unknown regions ─────────────────────────────
        unknown_mask = (out == -1).astype(np.uint8)
        if np.any(unknown_mask):
            dilated = cv2.dilate(unknown_mask, kernel_3x3, iterations=2).astype(bool)
            out[dilated] = -1

        # ─── Step 7: Publish ────────────────────────────────────────────
        out_msg = OccupancyGrid()
        out_msg.header = hm_msg.header
        out_msg.info = hm_msg.info
        out_msg.data = out.flatten().tolist()
        self.pub.publish(out_msg)

        elapsed = time.time() - start_time
        if self.first_publish:
            n_lethal = int(np.sum(all_lethal))
            n_inscribed = int(np.sum(inscribed_mask & ~lethal_mask))
            n_inflated = int(np.sum(inflation_band))
            self.get_logger().info(
                f"First unified costmap published ({elapsed:.3f}s). "
                f"Lethal: {n_lethal} (height-derived), "
                f"Inscribed inflation: {n_inscribed}, "
                f"Inflation band: {n_inflated}, "
                f"Unknown: {int(np.sum(out.flatten() == -1))}")
            self.first_publish = False


def main(args=None):
    rclpy.init(args=args)
    node = HeightmapToCostmap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
