#!/usr/bin/env python3
"""
Passive Global Controller Evaluator & RViz Visualizer.
- Subscribes dynamically to /plan (or /path), /odom, and /cmd_vel without hardcoded waypoints or fixed user paths.
- Spawns RViz2 with pre-configured TF, Odometry, Path, and 3D Robot Marker.
- Evaluates real-time trajectory tracking (lateral error, heading error, overshoot), velocity envelope limits, 
  jerk dynamics, and command signal smoothness.
- Generates HTML diagnostics report and matplotlib performance plots upon shutdown.
"""

import csv
import math
import os
import pathlib
import subprocess
import tempfile
import webbrowser
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

try:
    import numpy as np
except ImportError:
    np = None



class PassiveEvaluatorNode(Node):
    def __init__(self) -> None:
        super().__init__('passive_evaluator_node')

        # --- Parameters Configuration ---
        default_csv = os.path.join(tempfile.gettempdir(), 'controller_global_behavior.csv')
        default_plot = os.path.join(tempfile.gettempdir(), 'controller_global_behavior.png')
        default_rviz = os.path.join(tempfile.gettempdir(), 'evaluator_view.rviz')

        self.declare_parameter('csv_output_path', default_csv)
        self.declare_parameter('plot_output_path', default_plot)
        self.declare_parameter('rviz_config_path', default_rviz)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('launch_rviz', True)
        self.declare_parameter('publish_tf', False)

        # Dynamic Topic Names (No hardcoded paths or scenario assumptions)
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('case_topic', '/testing/current_case')
        self.declare_parameter('controller_param_file', '')

        # Physical Envelope & Performance Thresholds (Default matching MPPI vx_max=0.7 m/s, wz_max=1.2 rad/s)
        self.declare_parameter('velocity_limit_m_s', 0.7)
        self.declare_parameter('angular_velocity_limit_rad_s', 1.2)
        self.declare_parameter('max_allowed_jerk_deg_s2', 150.0)
        self.declare_parameter('instability_jerk_threshold', 300.0)
        self.declare_parameter('similarity_threshold_m', 0.05)
        self.declare_parameter('overshoot_threshold_m', 0.20)

        # Load parameters
        self.csv_output_path = str(self.get_parameter('csv_output_path').value)
        self.plot_output_path = str(self.get_parameter('plot_output_path').value)
        self.rviz_config_path = str(self.get_parameter('rviz_config_path').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.launch_rviz = bool(self.get_parameter('launch_rviz').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.path_topic = str(self.get_parameter('path_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.case_topic = str(self.get_parameter('case_topic').value)
        self.controller_param_file = str(self.get_parameter('controller_param_file').value)

        self.velocity_limit_m_s = float(self.get_parameter('velocity_limit_m_s').value)
        self.angular_velocity_limit_rad_s = float(self.get_parameter('angular_velocity_limit_rad_s').value)
        self.max_allowed_jerk_deg_s2 = float(self.get_parameter('max_allowed_jerk_deg_s2').value)
        self.instability_jerk_threshold = float(self.get_parameter('instability_jerk_threshold').value)
        self.similarity_threshold_m = float(self.get_parameter('similarity_threshold_m').value)
        self.overshoot_threshold_m = float(self.get_parameter('overshoot_threshold_m').value)

        # Automatically extract controller limits if a controller_param_file YAML is supplied
        if self.controller_param_file:
            self._load_controller_yaml(self.controller_param_file)

        # --- QoS and Subscriptions / Publishers ---
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_cb, self.qos_profile)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, self.qos_profile)
        self.cmd_sub = self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, self.qos_profile)
        if self.cmd_topic in ['/cmd_vel', 'cmd_vel']:
            self.fallback_sub1 = self.create_subscription(Twist, '/diffdrive_controller/cmd_vel_unstamped', self.cmd_cb, self.qos_profile)
            self.fallback_sub2 = self.create_subscription(Twist, '/nav2_cmd_vel', self.cmd_cb, self.qos_profile)
            self.fallback_sub3 = self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel', self.cmd_stamped_cb, self.qos_profile)
        self.case_sub = self.create_subscription(String, self.case_topic, self.case_cb, 10)

        self.marker_pub = self.create_publisher(Marker, '/robot_visual_marker', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Internal States & Statistics ---
        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Optional[Twist] = None
        self.current_case_id: str = 'DYNAMIC_RUN'

        self.start_time = None
        self.last_time = None

        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_cmd_accel_v = None
        self.last_cmd_accel_w = None

        self.last_actual_v = 0.0
        self.last_actual_w = 0.0
        self.last_actual_accel_v = None
        self.last_actual_accel_w = None

        # Cumulative Metrics
        self.total_samples = 0
        self.good_samples_count = 0
        self.linear_vel_violations = 0
        self.angular_vel_violations = 0
        self.unstable_jerk_samples = 0

        self.sum_sq_steering_jerk = 0.0
        self.sum_sq_linear_jerk = 0.0
        self.total_variation_v = 0.0
        self.total_variation_w = 0.0
        self.max_overshoot_m = 0.0

        self.rviz_process = None
        if self.launch_rviz:
            self._setup_and_launch_rviz()

        self._initialize_csv()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.loop_step)
        self.get_logger().info(
            f"Passive Evaluator Node started cleanly. Listening on Path='{self.path_topic}', "
            f"Odom='{self.odom_topic}', Cmd='{self.cmd_topic}', Limits: v_max={self.velocity_limit_m_s}m/s, w_max={self.angular_velocity_limit_rad_s}rad/s"
        )

    def _load_controller_yaml(self, file_path: str) -> None:
        if not file_path or not os.path.exists(file_path):
            return
        try:
            import yaml
            with open(file_path, 'r') as f:
                data = yaml.safe_load(f)

            def search_key(d, target_keys):
                if isinstance(d, dict):
                    for k, v in d.items():
                        if k in target_keys and isinstance(v, (int, float)):
                            return float(v)
                        res = search_key(v, target_keys)
                        if res is not None:
                            return res
                return None

            v_max = search_key(data, ['vx_max', 'max_vel_x', 'max_speed', 'velocity_limit', 'v_max'])
            if v_max is not None and v_max > 0:
                self.velocity_limit_m_s = v_max
                self.get_logger().info(f"Loaded linear velocity limit from '{file_path}': {self.velocity_limit_m_s} m/s")

            w_max = search_key(data, ['wz_max', 'max_vel_theta', 'angular_velocity_limit', 'w_max'])
            if w_max is not None and w_max > 0:
                self.angular_velocity_limit_rad_s = w_max
                self.get_logger().info(f"Loaded angular velocity limit from '{file_path}': {self.angular_velocity_limit_rad_s} rad/s")

            goal_tol = search_key(data, ['xy_goal_tolerance'])
            if goal_tol is not None and goal_tol > 0:
                self.similarity_threshold_m = goal_tol
                self.get_logger().info(f"Loaded goal tolerance threshold from '{file_path}': {self.similarity_threshold_m} m")

        except Exception as e:
            self.get_logger().warn(f"Failed to parse controller_param_file '{file_path}': {e}")

    def _setup_and_launch_rviz(self) -> None:
        """Generates a dynamic RViz display profile bound to the configured topics and launches RViz2."""
        rviz_content = f"""Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /TF1
        - /Target Path1
        - /Robot Odometry1
        - /Robot Chassis Marker1
      Splitter Ratio: 0.5
    Tree Height: 464
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Name: Tool Properties
    Splitter Ratio: 0.588679
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Value: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Axes: true
      Show Names: true
      Value: true
    - Class: rviz_default_plugins/Path
      Name: Target Path
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {self.path_topic}
      Color: 0; 255; 0
      Line Style: Lines
      Line Width: 0.05
      Value: true
    - Class: rviz_default_plugins/Odometry
      Name: Robot Odometry
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: {self.odom_topic}
      Shape: Arrow
      Color: 255; 0; 0
      Value: true
    - Class: rviz_default_plugins/Marker
      Name: Robot Chassis Marker
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_visual_marker
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785398
      Target Frame: odom
      Value: Orbit (rviz)
      Yaw: 0.785398
"""
        try:
            directory = os.path.dirname(self.rviz_config_path)
            if directory and not os.path.exists(directory):
                os.makedirs(directory, exist_ok=True)
            with open(self.rviz_config_path, 'w') as f:
                f.write(rviz_content.strip())
            self.rviz_process = subprocess.Popen(
                ['rviz2', '-d', self.rviz_config_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"RViz2 spawned cleanly with config: {self.rviz_config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to launch RViz2: {e}")

    def _initialize_csv(self) -> None:
        directory = os.path.dirname(self.csv_output_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)

        with open(self.csv_output_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'timestamp_s', 'case_id',
                'cmd_linear_v', 'actual_linear_v',
                'cmd_angular_w', 'actual_angular_w',
                'linear_jerk_m_s3', 'steering_jerk_deg_s2',
                'lat_error_m', 'pos_error_m', 'heading_error_deg', 'overshoot_m',
                'linear_violation', 'angular_violation', 'jerk_instability', 'similarity_ratio'
            ])

    def path_cb(self, msg: Path) -> None:
        self.latest_path = msg

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def cmd_cb(self, msg: Twist) -> None:
        self.latest_cmd = msg

    def cmd_stamped_cb(self, msg: TwistStamped) -> None:
        self.latest_cmd = msg.twist

    def case_cb(self, msg: String) -> None:
        self.current_case_id = msg.data

    def loop_step(self) -> None:
        if not rclpy.ok() or self.latest_odom is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        
        actual_v = self.latest_odom.twist.twist.linear.x
        actual_w = self.latest_odom.twist.twist.angular.z

        if self.latest_cmd:
            cmd_v = self.latest_cmd.linear.x
            cmd_w = self.latest_cmd.angular.z
        else:
            cmd_v = actual_v
            cmd_w = actual_w

        if self.start_time is None:
            self.start_time = now
            self.last_time = now
            self.last_actual_v = actual_v
            self.last_actual_w = actual_w
            self.last_cmd_v = cmd_v
            self.last_cmd_w = cmd_w
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return

        # 1. Kinematics from Odometry
        rx = self.latest_odom.pose.pose.position.x
        ry = self.latest_odom.pose.pose.position.y
        raw_q = self.latest_odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (raw_q.w * raw_q.z + raw_q.x * raw_q.y),
            1.0 - 2.0 * (raw_q.y * raw_q.y + raw_q.z * raw_q.z)
        )

        # 2. RViz Visual Markers & TF
        self._publish_robot_visuals(rx, ry, raw_q)

        # 3. Path Metrics: Continuous Segment Projection
        lat_error_m, heading_error_rad, pos_error_m = self.compute_path_metrics(self.latest_path, rx, ry, yaw)
        heading_error_deg = math.degrees(heading_error_rad)

        # 4. Motion Smoothness & Jerk Calculations (2nd derivative of velocity)
        # --- Actual Motion Dynamics (Odometry Feedback) ---
        delta_act_v = actual_v - self.last_actual_v
        delta_act_w = actual_w - self.last_actual_w

        act_accel_v = delta_act_v / dt
        act_accel_w = delta_act_w / dt

        if self.last_actual_accel_v is not None:
            actual_linear_jerk = (act_accel_v - self.last_actual_accel_v) / dt
        else:
            actual_linear_jerk = 0.0

        if self.last_actual_accel_w is not None:
            actual_angular_jerk_rad = (act_accel_w - self.last_actual_accel_w) / dt
            actual_angular_jerk_deg = math.degrees(actual_angular_jerk_rad)
        else:
            actual_angular_jerk_deg = 0.0

        # --- Commanded Signal Dynamics (Control Command) ---
        delta_cmd_v = cmd_v - self.last_cmd_v
        delta_cmd_w = cmd_w - self.last_cmd_w

        cmd_accel_v = delta_cmd_v / dt
        cmd_accel_w = delta_cmd_w / dt

        if self.last_cmd_accel_v is not None:
            cmd_linear_jerk = (cmd_accel_v - self.last_cmd_accel_v) / dt
        else:
            cmd_linear_jerk = 0.0

        if self.last_cmd_accel_w is not None:
            cmd_angular_jerk_deg = math.degrees((cmd_accel_w - self.last_cmd_accel_w) / dt)
        else:
            cmd_angular_jerk_deg = 0.0

        # Primary Jerk metrics logged (selecting actual motion jerk, or max of act/cmd)
        if self.latest_cmd is None:
            linear_jerk = actual_linear_jerk
            steering_jerk_deg = actual_angular_jerk_deg
        else:
            linear_jerk = actual_linear_jerk if abs(actual_linear_jerk) >= abs(cmd_linear_jerk) else cmd_linear_jerk
            steering_jerk_deg = actual_angular_jerk_deg if abs(actual_angular_jerk_deg) >= abs(cmd_angular_jerk_deg) else cmd_angular_jerk_deg

        # Cumulative Smoothness Statistics
        self.sum_sq_steering_jerk += steering_jerk_deg ** 2
        self.sum_sq_linear_jerk += linear_jerk ** 2
        self.total_variation_v += abs(delta_act_v)
        self.total_variation_w += abs(delta_act_w)

        # Overshoot Calculation (perpendicular deviation during turns)
        is_turning = abs(actual_w) > 0.05 or abs(cmd_w) > 0.05
        overshoot_m = max(0.0, lat_error_m - self.overshoot_threshold_m) if is_turning else 0.0
        if is_turning:
            self.max_overshoot_m = max(self.max_overshoot_m, lat_error_m)

        # Envelope & Instability Flags
        v_viol = (abs(actual_v) > self.velocity_limit_m_s) or (abs(cmd_v) > self.velocity_limit_m_s)
        w_viol = (abs(actual_w) > self.angular_velocity_limit_rad_s) or (abs(cmd_w) > self.angular_velocity_limit_rad_s)
        jerk_instable = abs(steering_jerk_deg) > self.instability_jerk_threshold

        self.total_samples += 1
        if abs(lat_error_m) <= self.similarity_threshold_m:
            self.good_samples_count += 1
        if v_viol:
            self.linear_vel_violations += 1
        if w_viol:
            self.angular_vel_violations += 1
        if jerk_instable:
            self.unstable_jerk_samples += 1

        similarity_ratio = self.good_samples_count / self.total_samples

        # 5. Log to CSV
        with open(self.csv_output_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                round(now - self.start_time, 4), self.current_case_id,
                round(cmd_v, 4), round(actual_v, 4),
                round(cmd_w, 4), round(actual_w, 4),
                round(linear_jerk, 4), round(steering_jerk_deg, 2),
                round(lat_error_m, 6), round(pos_error_m, 6), round(heading_error_deg, 4), round(overshoot_m, 6),
                int(v_viol), int(w_viol), int(jerk_instable), round(similarity_ratio, 4)
            ])

        self.last_actual_accel_v = act_accel_v
        self.last_actual_accel_w = act_accel_w
        self.last_actual_v = actual_v
        self.last_actual_w = actual_w

        self.last_cmd_accel_v = cmd_accel_v
        self.last_cmd_accel_w = cmd_accel_w
        self.last_cmd_v = cmd_v
        self.last_cmd_w = cmd_w
        self.last_time = now

    def _publish_robot_visuals(self, rx: float, ry: float, orientation: Quaternion) -> None:
        if not rclpy.ok():
            return
        now_msg = self.get_clock().now().to_msg()

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = rx
            t.transform.translation.y = ry
            t.transform.translation.z = 0.0
            t.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(t)

        marker = Marker()
        marker.header.stamp = now_msg
        marker.header.frame_id = 'odom'
        marker.ns = 'robot_chassis'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = rx
        marker.pose.position.y = ry
        marker.pose.position.z = 0.2
        marker.pose.orientation = orientation
        marker.scale.x = 0.6
        marker.scale.y = 0.4
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def compute_path_metrics(self, path: Optional[Path], rx: float, ry: float, yaw: float) -> Tuple[float, float, float]:
        """
        Calculates exact continuous lateral error (m), heading error (rad),
        and Euclidean position error (m) relative to nearest line segment on target Path.
        """
        if not path or len(path.poses) == 0:
            return 0.0, 0.0, 0.0

        if len(path.poses) == 1:
            px = path.poses[0].pose.position.x
            py = path.poses[0].pose.position.y
            dist = math.hypot(px - rx, py - ry)
            return dist, 0.0, dist

        min_dist_sq = float('inf')
        best_lat_error = 0.0
        best_heading_error = 0.0

        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position

            x1, y1 = p1.x, p1.y
            x2, y2 = p2.x, p2.y

            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy

            if seg_len_sq < 1e-9:
                t = 0.0
            else:
                t = max(0.0, min(1.0, ((rx - x1) * dx + (ry - y1) * dy) / seg_len_sq))

            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            dist_sq = (rx - proj_x) ** 2 + (ry - proj_y) ** 2

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq

                lat_error = abs((proj_x - rx) * math.sin(yaw) - (proj_y - ry) * math.cos(yaw))
                best_lat_error = lat_error

                seg_yaw = math.atan2(dy, dx)
                diff = seg_yaw - yaw
                best_heading_error = math.atan2(math.sin(diff), math.cos(diff))

        return best_lat_error, best_heading_error, math.sqrt(min_dist_sq)

    def _save_plot(self) -> None:
        if plt is None or not os.path.exists(self.csv_output_path):
            return
        try:
            with open(self.csv_output_path, 'r') as f:
                data = list(csv.DictReader(f))
            if not data:
                return

            times = [float(r['timestamp_s']) for r in data]
            cmd_v = [float(r['cmd_linear_v']) for r in data]
            act_v = [float(r['actual_linear_v']) for r in data]
            linear_jerk = [float(r['linear_jerk_m_s3']) for r in data]
            steering_jerk = [float(r['steering_jerk_deg_s2']) for r in data]
            lat_err = [float(r['lat_error_m']) for r in data]
            overshoot = [float(r['overshoot_m']) for r in data]

            fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

            # Subplot 1: Linear Velocity Tracking
            ax1.plot(times, cmd_v, label='Cmd Linear Velocity (m/s)', color='#1f77b4', linewidth=1.8)
            ax1.plot(times, act_v, label='Actual Linear Velocity (m/s)', color='#ff7f0e', linestyle='--', linewidth=1.5)
            ax1.axhline(self.velocity_limit_m_s, color='red', linestyle=':', label='Velocity Limit')
            ax1.set_ylabel('Velocity (m/s)')
            ax1.set_title('Controller Velocity, Jerk & Smoothness Performance')
            ax1.legend(loc='upper right')
            ax1.grid(True, linestyle='--', alpha=0.6)

            # Subplot 2: Jerk Smoothness Metrics
            ax2.plot(times, steering_jerk, label='Steering Jerk (deg/s³)', color='#9467bd', linewidth=1.5)
            ax2.plot(times, linear_jerk, label='Linear Jerk (m/s³)', color='#2ca02c', linestyle=':', linewidth=1.2)
            ax2.axhline(self.max_allowed_jerk_deg_s2, color='orange', linestyle='--', label='Jerk Warning Limit')
            ax2.set_ylabel('Jerk')
            ax2.legend(loc='upper right')
            ax2.grid(True, linestyle='--', alpha=0.6)

            # Subplot 3: Lateral Error & Overshoot
            ax3.plot(times, lat_err, label='Lateral Error (m)', color='#d62728', linewidth=1.5)
            ax3.plot(times, overshoot, label='Turn Overshoot (m)', color='#e377c2', linestyle='--', linewidth=1.2)
            ax3.set_ylabel('Error (m)')
            ax3.set_xlabel('Session Time (s)')
            ax3.legend(loc='upper right')
            ax3.grid(True, linestyle='--', alpha=0.6)

            plt.tight_layout()
            plt.savefig(self.plot_output_path, dpi=150)
            plt.close()
            print(f"\n[INFO] Diagnostic plots saved to: file://{pathlib.Path(self.plot_output_path).resolve()}")
        except Exception as e:
            self.get_logger().error(f"Plot generation failed: {e}")

    def _generate_html_report(self) -> None:
        report_path = os.path.splitext(self.csv_output_path)[0] + '_report.html'
        rows = []
        if os.path.exists(self.csv_output_path):
            with open(self.csv_output_path, 'r') as f:
                rows = list(csv.DictReader(f))

        n = len(rows)
        if n > 0:
            lat_errors = [abs(float(r['lat_error_m'])) for r in rows]
            heading_errors = [abs(float(r['heading_error_deg'])) for r in rows]
            steering_jerks = [abs(float(r['steering_jerk_deg_s2'])) for r in rows]
            avg_lat, max_lat = sum(lat_errors) / n, max(lat_errors)
            avg_heading = sum(heading_errors) / n
            avg_steering_jerk = sum(steering_jerks) / n
            max_steering_jerk = max(steering_jerks)
            rms_steering_jerk = math.sqrt(self.sum_sq_steering_jerk / max(1, n))
            rms_linear_jerk = math.sqrt(self.sum_sq_linear_jerk / max(1, n))
            final_similarity = float(rows[-1]['similarity_ratio'])
        else:
            avg_lat = max_lat = avg_heading = avg_steering_jerk = max_steering_jerk = 0.0
            rms_steering_jerk = rms_linear_jerk = final_similarity = 0.0

        html_content = f"""<!DOCTYPE html>
<html>
<head>
    <title>Controller Motion & Smoothness Diagnostics</title>
    <style>
        body {{ font-family: 'Segoe UI', Tahoma, Arial, sans-serif; margin: 30px; background: #f4f4f9; color: #1a1a1a; }}
        .card {{ background: white; padding: 22px; border-radius: 8px; box-shadow: 0 3px 6px rgba(0,0,0,0.08); margin-bottom: 22px; border-top: 4px solid #0056b3; }}
        h1 {{ color: #0056b3; margin-bottom: 20px; }}
        h3 {{ margin-top: 0; color: #222; border-bottom: 2px solid #eee; padding-bottom: 8px; }}
        .grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 15px; }}
        .metric {{ font-size: 15px; margin: 8px 0; color: #333; }}
        .highlight {{ font-weight: bold; color: #0056b3; }}
        .warning {{ color: #d9534f; font-weight: bold; }}
    </style>
</head>
<body>
    <h1>Dynamic Controller Performance & Smoothness Report</h1>
    
    <div class="card">
        <h3>1. Trajectory & Path Accuracy</h3>
        <div class="grid">
            <div>
                <div class="metric"><strong>Path Tracking Similarity:</strong> <span class="highlight">{final_similarity * 100:.2f}%</span></div>
                <div class="metric"><strong>Average Lateral Error:</strong> {avg_lat:.4f} m</div>
                <div class="metric"><strong>Peak Lateral Error:</strong> {max_lat:.4f} m</div>
            </div>
            <div>
                <div class="metric"><strong>Average Heading Error:</strong> {avg_heading:.2f}°</div>
                <div class="metric"><strong>Maximum Turn Overshoot:</strong> {self.max_overshoot_m:.4f} m</div>
            </div>
        </div>
    </div>

    <div class="card">
        <h3>2. Signal Smoothness & Jerk Metrics</h3>
        <div class="grid">
            <div>
                <div class="metric"><strong>RMS Steering Jerk:</strong> <span class="highlight">{rms_steering_jerk:.2f} deg/s³</span></div>
                <div class="metric"><strong>Peak Steering Jerk:</strong> {max_steering_jerk:.2f} deg/s³</div>
                <div class="metric"><strong>Average Steering Jerk:</strong> {avg_steering_jerk:.2f} deg/s³</div>
            </div>
            <div>
                <div class="metric"><strong>RMS Linear Jerk:</strong> {rms_linear_jerk:.4f} m/s³</div>
                <div class="metric"><strong>Total Linear Variation (Chatter):</strong> {self.total_variation_v:.3f} m/s</div>
                <div class="metric"><strong>Total Steering Variation (Chatter):</strong> {self.total_variation_w:.3f} rad/s</div>
            </div>
        </div>
    </div>

    <div class="card">
        <h3>3. Velocity & Envelope Safety</h3>
        <div class="grid">
            <div>
                <div class="metric"><strong>Linear Velocity Breaches:</strong> <span class="{ 'warning' if self.linear_vel_violations > 0 else 'highlight' }">{self.linear_vel_violations}</span></div>
                <div class="metric"><strong>Angular Velocity Breaches:</strong> <span class="{ 'warning' if self.angular_vel_violations > 0 else 'highlight' }">{self.angular_vel_violations}</span></div>
            </div>
            <div>
                <div class="metric"><strong>Unstable Jerk Instabilities:</strong> <span class="{ 'warning' if self.unstable_jerk_samples > 0 else 'highlight' }">{self.unstable_jerk_samples}</span></div>
                <div class="metric"><strong>Total Samples Analyzed:</strong> {n}</div>
            </div>
        </div>
    </div>
</body>
</html>"""
        try:
            with open(report_path, 'w') as f:
                f.write(html_content)
            full_uri = pathlib.Path(report_path).resolve().as_uri()
            print(f"\n[INFO] HTML Evaluation Report saved to: file://{pathlib.Path(report_path).resolve()}")
            try:
                webbrowser.open(full_uri)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().error(f"HTML report generation failed: {e}")

    def _save_numpy_data(self) -> None:
        if np is None or not os.path.exists(self.csv_output_path):
            return
        npy_path = os.path.splitext(self.csv_output_path)[0] + '.npy'
        npz_path = os.path.splitext(self.csv_output_path)[0] + '.npz'
        try:
            dtype_spec = [
                ('timestamp_s', 'f8'), ('case_id', 'U32'),
                ('cmd_linear_v', 'f4'), ('actual_linear_v', 'f4'),
                ('cmd_angular_w', 'f4'), ('actual_angular_w', 'f4'),
                ('linear_jerk_m_s3', 'f4'), ('steering_jerk_deg_s2', 'f4'),
                ('lat_error_m', 'f4'), ('pos_error_m', 'f4'), ('heading_error_deg', 'f4'), ('overshoot_m', 'f4'),
                ('linear_violation', 'i4'), ('angular_violation', 'i4'), ('jerk_instability', 'i4'), ('similarity_ratio', 'f4')
            ]
            data_struct = np.genfromtxt(self.csv_output_path, delimiter=',', names=True, dtype=dtype_spec, encoding='utf-8')
            np.save(npy_path, data_struct)
            npz_dict = {name: data_struct[name] for name in data_struct.dtype.names}
            np.savez_compressed(npz_path, **npz_dict)
            print(f"[INFO] NumPy array saved to: file://{pathlib.Path(npy_path).resolve()}")
            print(f"[INFO] Compressed NumPy archive saved to: file://{pathlib.Path(npz_path).resolve()}")
        except Exception as e:
            self.get_logger().error(f"NumPy export failed: {e}")

    def destroy_node(self) -> None:
        self._save_plot()
        self._generate_html_report()
        self._save_numpy_data()
        if self.rviz_process:
            self.rviz_process.terminate()
        super().destroy_node()



def main(args=None) -> None:
    rclpy.init(args=args)
    node = PassiveEvaluatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()