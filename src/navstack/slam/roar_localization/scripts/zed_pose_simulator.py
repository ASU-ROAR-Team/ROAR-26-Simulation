#!/usr/bin/env python3
"""
zed_pose_simulator.py
=====================
Simulates the ZED2i positional-tracking stream from clean ground truth.

Real ZED2i VIO in normal conditions (well-lit, good visual features) is
very accurate — sub-cm per frame with slow drift. This simulator matches
that: it adds SMALL realistic noise that the IESKF can trust and fuse
effectively. It does NOT simulate degraded modes (glare, dust, tracking
loss) — those are separate stress-test scenarios.

Noise model:
  1. Per-sample position white noise:  σ = 2 mm
  2. Slow position drift:              random walk σ = 0.1 mm/sample
     (grows ~1 cm per 100 samples ≈ 3 seconds at 30 Hz)
  3. Position drift cap:               5 cm (SDK relocalizes before this)
  4. Yaw white noise:                  σ = 1 mrad
  5. Reported covariance:              matches actual noise (small, honest)

All parameters are declared for easy tuning. Timestamps are preserved
from the input (sim-time consistent).
"""

import math
import random
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class ZedPoseSimulator(Node):
    def __init__(self):
        super().__init__("zed_pose_simulator")

        # ── Topics ─────────────────────────────────────────────────────────
        self.input_topic  = self.declare_parameter(
            "input_topic",  "/ground_truth/pose").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/zed2i/zed_node/pose_with_covariance").value

        # ── Realistic ZED2i noise (small, honest) ──────────────────────────
        # Per-sample position white noise (m)
        self.pos_white_sigma = float(self.declare_parameter(
            "pos_white_sigma", 0.002).value)          # 2 mm

        # Drift random walk (m per sample)
        self.drift_walk_sigma = float(self.declare_parameter(
            "drift_walk_sigma", 0.0001).value)        # 0.1 mm

        # Cap on drift magnitude (m)
        self.drift_cap_m = float(self.declare_parameter(
            "drift_cap_m", 0.05).value)               # 5 cm

        # Yaw white noise (rad)
        self.yaw_white_sigma = float(self.declare_parameter(
            "yaw_white_sigma", 0.001).value)          # ~0.06°

        # Reported covariance (should reflect actual noise honestly)
        self.reported_pos_var = float(self.declare_parameter(
            "reported_pos_var", 4.0e-6).value)        # σ² = (2mm)²

        self.reported_yaw_var = float(self.declare_parameter(
            "reported_yaw_var", 1.0e-6).value)

        # RNG
        self.seed = int(self.declare_parameter("seed", 42).value)
        random.seed(self.seed)

        # ── Internal state ─────────────────────────────────────────────────
        self.drift_x = 0.0
        self.drift_y = 0.0
        self.drift_z = 0.0

        # ── I/O ────────────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, self.input_topic, self._cb, 30)
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 30)

        self.get_logger().info(
            f"zed_pose_simulator up. {self.input_topic} -> {self.output_topic}")
        self.get_logger().info(
            f"  pos noise sigma = {self.pos_white_sigma*1000:.1f} mm  "
            f"drift walk sigma = {self.drift_walk_sigma*1000:.2f} mm/sample  "
            f"drift cap = {self.drift_cap_m*100:.0f} cm  "
            f"yaw noise sigma = {self.yaw_white_sigma*1000:.2f} mrad")

    def _cb(self, msg: PoseWithCovarianceStamped):
        # Update drift random walk
        self.drift_x += random.gauss(0.0, self.drift_walk_sigma)
        self.drift_y += random.gauss(0.0, self.drift_walk_sigma)
        self.drift_z += random.gauss(0.0, self.drift_walk_sigma * 0.3)

        # Cap drift magnitude
        mag = math.sqrt(self.drift_x**2 + self.drift_y**2 + self.drift_z**2)
        if mag > self.drift_cap_m:
            k = self.drift_cap_m / mag
            self.drift_x *= k
            self.drift_y *= k
            self.drift_z *= k

        # Read clean ground truth
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        gz = msg.pose.pose.position.z

        # Add drift + white noise
        sx = gx + self.drift_x + random.gauss(0.0, self.pos_white_sigma)
        sy = gy + self.drift_y + random.gauss(0.0, self.pos_white_sigma)
        sz = gz + self.drift_z + random.gauss(0.0, self.pos_white_sigma * 0.3)

        # Small yaw perturbation
        yaw_noise = random.gauss(0.0, self.yaw_white_sigma)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        half = yaw_noise * 0.5
        c = math.cos(half)
        s = math.sin(half)
        qx_out = c * qx - s * qy
        qy_out = c * qy + s * qx
        qz_out = c * qz + s * qw
        qw_out = c * qw - s * qz
        n = math.sqrt(qx_out**2 + qy_out**2 + qz_out**2 + qw_out**2)
        if n > 0:
            qx_out /= n
            qy_out /= n
            qz_out /= n
            qw_out /= n

        # Build output — preserve input timestamp for sim-time consistency
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        # "odom": matches ground_truth_bridge.cpp's ZED pose stamping. A ZED
        # visual-odometry pose is a continuous, drifting local estimate — it
        # belongs in odom, not map (REP-105) — and the two sim sources for
        # this topic previously disagreed ("map" here vs "odom" there).
        out.header.frame_id = "odom"
        out.pose.pose.position.x = sx
        out.pose.pose.position.y = sy
        out.pose.pose.position.z = sz
        out.pose.pose.orientation.x = qx_out
        out.pose.pose.orientation.y = qy_out
        out.pose.pose.orientation.z = qz_out
        out.pose.pose.orientation.w = qw_out

        # Honest covariance (matches actual noise)
        cov = [0.0] * 36
        cov[0]  = self.reported_pos_var
        cov[7]  = self.reported_pos_var
        cov[14] = self.reported_pos_var * 0.3
        cov[21] = self.reported_yaw_var
        cov[28] = self.reported_yaw_var
        cov[35] = self.reported_yaw_var
        out.pose.covariance = cov

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ZedPoseSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
