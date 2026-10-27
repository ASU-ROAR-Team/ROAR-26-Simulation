#!/usr/bin/env python3
"""
obstacle_simulator.py
=====================
SIMULATION-ONLY node that fakes the perception team's obstacle detector,
feeding the mapping module (terrain_mapping) with ObstacleDetectionArray
messages.

How it works
------------
1. Loads virtual obstacle definitions (position + bbox dims) from the
   scenario YAML. These obstacles do NOT exist in Gazebo — they are virtual
   targets for testing the mapping pipeline.
2. Subscribes to the rover ground-truth pose from Gazebo
   (/world/marsyard/pose/info, PoseArray, pose_index for roar_rover).
3. On a timer (default 5 Hz), checks each obstacle against:
     - detection range  (max_range)
     - camera field of view (horizontal, approximated as a cone around the
       rover's forward axis)
4. Publishes detections with header.frame_id = "map".

WHY frame_id="map" MATTERS:
   terrain_mapping's _transform_to_map_frame() short-circuits when the source
   frame equals the map frame — no TF lookup happens. This bypasses the
   whole TF problem in simulation (nobody publishes map→odom, and diff_drive
   owns odom→base_link). The real perception node will publish in the camera
   frame and TF wiring will be solved for the real rover.

Noise: Gaussian noise is added to the detected position (σ configurable).
NOTE terrain_mapping tracks each obstacle with a Kalman filter: re-sightings
refine the position estimate, and an obstacle only enters the published maps
after enough consistent sightings verify it — so obstacles must be
re-detected at least twice to appear.
"""

import math
import random

import rclpy
import yaml
from geometry_msgs.msg import PoseArray
from rclpy.node import Node

from active_mapping_msgs.msg import ObstacleDetection, ObstacleDetectionArray


class ObstacleSimulator(Node):

    def __init__(self):
        super().__init__("obstacle_simulator")

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("ground_truth_topic", "/world/marsyard/pose/info")
        self.declare_parameter("pose_index", 11)
        self.declare_parameter("obstacle_topic", "/detections/obstacles")
        self.declare_parameter("detection_rate_hz", 5.0)
        self.declare_parameter("max_range", 6.0)          # [m]
        self.declare_parameter("horizontal_fov_deg", 90.0)
        self.declare_parameter("noise_sigma", 0.03)       # [m]
        self.declare_parameter("noise_seed", 7)
        self.declare_parameter("map_frame", "map")

        scenario_file = self.get_parameter("scenario_file").value
        self._gt_topic = self.get_parameter("ground_truth_topic").value
        self._pose_index = self.get_parameter("pose_index").value
        obstacle_topic = self.get_parameter("obstacle_topic").value
        rate = self.get_parameter("detection_rate_hz").value
        self._max_range = self.get_parameter("max_range").value
        self._cos_half_fov = math.cos(
            math.radians(self.get_parameter("horizontal_fov_deg").value) / 2.0)
        self._noise_sigma = self.get_parameter("noise_sigma").value
        self._map_frame = self.get_parameter("map_frame").value

        random.seed(self.get_parameter("noise_seed").value)

        # ── Load virtual obstacles from scenario ──────────────────────────────
        self._obstacles = []
        if scenario_file:
            try:
                with open(scenario_file, "r") as f:
                    data = yaml.safe_load(f)
                self._obstacles = data.get("obstacles", [])
                self.get_logger().info(
                    f"Loaded {len(self._obstacles)} virtual obstacles "
                    f"from {scenario_file}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to load scenario: {exc}")
        else:
            self.get_logger().error(
                "No scenario_file given — publishing nothing.")

        # ── Rover ground truth ─────────────────────────────────────────────────
        self._rover_xy = None
        self._rover_yaw = None
        self.create_subscription(
            PoseArray, self._gt_topic, self._on_ground_truth, 10)

        # ── Publisher + timer ──────────────────────────────────────────────────
        self._pub = self.create_publisher(
            ObstacleDetectionArray, obstacle_topic, 10)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"ObstacleSimulator: gt={self._gt_topic}[{self._pose_index}] "
            f"out={obstacle_topic} range={self._max_range:.1f}m "
            f"rate={rate:.1f}Hz noise={self._noise_sigma:.3f}m "
            f"frame={self._map_frame}")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_ground_truth(self, msg: PoseArray):
        if self._pose_index >= len(msg.poses):
            return
        pose = msg.poses[self._pose_index]
        self._rover_xy = (pose.position.x, pose.position.y)
        q = pose.orientation
        self._rover_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_timer(self):
        if self._rover_xy is None or not self._obstacles:
            return

        rx, ry = self._rover_xy
        yaw = self._rover_yaw
        fwd = (math.cos(yaw), math.sin(yaw))

        out = ObstacleDetectionArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._map_frame

        for obs in self._obstacles:
            dx = obs["x"] - rx
            dy = obs["y"] - ry
            dist = math.hypot(dx, dy)

            # Range gate
            if dist > self._max_range or dist < 1e-3:
                continue

            # FOV gate: angle between forward axis and obstacle bearing
            cos_angle = (dx * fwd[0] + dy * fwd[1]) / dist
            if cos_angle < self._cos_half_fov:
                continue

            det = ObstacleDetection()
            det.header = out.header
            det.position.x = obs["x"] + random.gauss(0.0, self._noise_sigma)
            det.position.y = obs["y"] + random.gauss(0.0, self._noise_sigma)
            det.position.z = obs["dim_z"] / 2.0
            det.dimensions.x = float(obs["dim_x"])
            det.dimensions.y = float(obs["dim_y"])
            det.dimensions.z = float(obs["dim_z"])
            # Confidence grows as the rover gets closer
            det.confidence = float(max(0.3, 1.0 - dist / self._max_range))
            det.class_id = int(obs.get("class_id", 1))
            out.detections.append(det)

        # Publish even if empty (downstream can use timestamps)
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
