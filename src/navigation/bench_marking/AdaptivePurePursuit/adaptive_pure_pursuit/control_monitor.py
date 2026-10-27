#!/usr/bin/env python3
"""Continuous Global Metrics Evaluator for Autonomous Controllers."""

import csv
import math
import os
import webbrowser
import pathlib
import base64
import subprocess
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from std_msgs.msg import String

try:
    from rclpy.executors import ExternalShutdownException
except ImportError:
    ExternalShutdownException = ()

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


class ControllerGlobalMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__('controller_global_monitor_node')

        # Configurable Paths
        self.declare_parameter('csv_output_path', '/tmp/controller_global_behavior.csv')
        self.declare_parameter('plot_output_path', '/tmp/controller_global_behavior.png')
        self.declare_parameter('publish_rate_hz', 50.0)
        
        # Hard Physical Thresholds (Controller Envelope)
        self.declare_parameter('velocity_limit_m_s', 1.0)
        self.declare_parameter('angular_velocity_limit_rad_s', 1.5)
        self.declare_parameter('max_allowed_jerk_deg_s2', 150.0)
        self.declare_parameter('instability_jerk_threshold', 300.0)

        # Path Evaluation Parameters
        self.declare_parameter('overshoot_threshold_m', 0.2)
        self.declare_parameter('similarity_threshold_m', 0.05)
        self.declare_parameter('rotation_detection_threshold_rad_s', 0.05)

        self.csv_output_path = self.get_parameter('csv_output_path').value
        self.plot_output_path = self.get_parameter('plot_output_path').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        
        self.velocity_limit_m_s = float(self.get_parameter('velocity_limit_m_s').value)
        self.angular_velocity_limit_rad_s = float(self.get_parameter('angular_velocity_limit_rad_s').value)
        self.max_allowed_jerk_deg_s2 = float(self.get_parameter('max_allowed_jerk_deg_s2').value)
        self.instability_jerk_threshold = float(self.get_parameter('instability_jerk_threshold').value)

        self.overshoot_threshold_m = float(self.get_parameter('overshoot_threshold_m').value)
        self.similarity_threshold_m = float(self.get_parameter('similarity_threshold_m').value)
        self.rotation_detection_threshold_rad_s = float(self.get_parameter('rotation_detection_threshold_rad_s').value)

        # Monitor core topics directly to assess control signals vs real execution
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('case_topic', '/testing/current_case')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('robot_radius_m', 0.25)

        self.robot_radius_m = float(self.get_parameter('robot_radius_m').value)

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.path_sub = self.create_subscription(Path, self.get_parameter('path_topic').value, self.path_cb, self.qos_profile)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, self.qos_profile)
        self.cmd_sub = self.create_subscription(Twist, self.get_parameter('cmd_topic').value, self.cmd_cb, self.qos_profile)
        self.case_sub = self.create_subscription(String, self.get_parameter('case_topic').value, self.case_cb, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.get_parameter('costmap_topic').value, self.costmap_cb, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.sample_and_log)

        # Global state variables
        self.latest_path: Optional[Path] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_cmd: Optional[Twist] = None
        self.latest_case_id: str = 'UNKNOWN'
        self.latest_costmap: Optional[OccupancyGrid] = None
        
        self.start_time = None
        self.last_time = None
        self.last_steering_cmd = 0.0
        self._has_destroyed = False

        # Global Running Statistics
        self.total_samples = 0
        self.good_samples_count = 0
        self.linear_vel_violations = 0
        self.angular_vel_violations = 0
        self.unstable_jerk_samples = 0
        self.accumulated_control_energy = 0.0  # Integral of absolute control efforts

        self._initialize_csv()
        self.get_logger().info(f"Global Controller Monitor Started. CSV: {self.csv_output_path}")

    def _initialize_csv(self) -> None:
        directory = os.path.dirname(self.csv_output_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        if os.path.exists(self.csv_output_path):
            os.remove(self.csv_output_path)

        with open(self.csv_output_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                'timestamp_s', 'case_id',
                'cmd_linear_x', 'actual_linear_x',
                'cmd_angular_z', 'actual_angular_z',
                'steering_jerk_deg_s2',
                'lat_error_m', 'pos_error_m', 'heading_error_rad', 'overshoot_m',
                'linear_violation', 'angular_violation', 'jerk_instability', 'similarity_ratio',
                'collision', 'min_clearance_m'
            ])

    def path_cb(self, msg: Path) -> None: self.latest_path = msg
    def odom_cb(self, msg: Odometry) -> None: self.latest_odom = msg
    def cmd_cb(self, msg: Twist) -> None: self.latest_cmd = msg
    def case_cb(self, msg: String) -> None: self.latest_case_id = msg.data
    def costmap_cb(self, msg: OccupancyGrid) -> None: self.latest_costmap = msg

    def sample_and_log(self) -> None:
        if self.latest_odom is None or self.latest_cmd is None or self.latest_path is None:
            return

        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.start_time is None:
            self.start_time = current_time
            self.last_time = current_time
            self.last_steering_cmd = self.latest_cmd.angular.z
            return

        dt = current_time - self.last_time
        if dt <= 0.0: return

        # Read actual robot response states
        actual_linear = self.latest_odom.twist.twist.linear.x
        actual_angular = self.latest_odom.twist.twist.angular.z

        # Read target outputs from the controller
        cmd_linear = self.latest_cmd.linear.x
        cmd_angular = self.latest_cmd.angular.z

        # Geometric Calculations
        lat_error_m, pos_error_m = self.compute_errors(self.latest_path, self.latest_odom)
        heading_error_rad = self.compute_heading_error(self.latest_path, self.latest_odom)

        # Control derivative analysis (Smoothness / Jerk Profile)
        delta_steering = cmd_angular - self.last_steering_cmd
        steering_jerk_deg_s2 = math.degrees(delta_steering / dt)

        # Run-time Envelope Violations
        v_viol = abs(cmd_linear) > self.velocity_limit_m_s
        w_viol = abs(cmd_angular) > self.angular_velocity_limit_rad_s
        jerk_instable = abs(steering_jerk_deg_s2) > self.instability_jerk_threshold

        # Overshoot Calculation
        overshoot_m = 0.0
        if abs(cmd_angular) > self.rotation_detection_threshold_rad_s:
            if abs(lat_error_m) > self.overshoot_threshold_m:
                overshoot_m = abs(lat_error_m) - self.overshoot_threshold_m

        # Update Globals
        self.total_samples += 1
        if abs(lat_error_m) <= self.similarity_threshold_m:
            self.good_samples_count += 1
        if v_viol: self.linear_vel_violations += 1
        if w_viol: self.angular_vel_violations += 1
        if jerk_instable: self.unstable_jerk_samples += 1
        self.accumulated_control_energy += (abs(cmd_linear) + abs(cmd_angular)) * dt

        similarity_ratio = self.good_samples_count / self.total_samples

        # Log standalone data vectors
        with open(self.csv_output_path, 'a', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow([
                round(current_time - self.start_time, 4), self.latest_case_id,
                round(cmd_linear, 4), round(actual_linear, 4),
                round(cmd_angular, 4), round(actual_angular, 4),
                round(steering_jerk_deg_s2, 2),
                round(lat_error_m, 6), round(pos_error_m, 6), round(heading_error_rad, 6), round(overshoot_m, 6),
                int(v_viol), int(w_viol), int(jerk_instable), round(similarity_ratio, 4),
                int(self._check_collision()), round(self._min_clearance(), 3)
            ])

        self.last_steering_cmd = cmd_angular
        self.last_time = current_time

    def compute_errors(self, path: Path, odom: Odometry) -> tuple:
        if not path.poses: return 0.0, 0.0
        robot_x, robot_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        yaw = self.quaternion_to_yaw(odom.pose.pose.orientation)

        best = None
        for p in path.poses:
            dist = math.hypot(p.pose.position.x - robot_x, p.pose.position.y - robot_y)
            if best is None or dist < best[0]:
                best = (dist, p.pose.position.x, p.pose.position.y)

        if best is None: return 0.0, 0.0
        lat_error = abs((best[1] - robot_x) * math.sin(yaw) - (best[2] - robot_y) * math.cos(yaw))
        return lat_error, best[0]

    def compute_heading_error(self, path: Path, odom: Odometry) -> float:
        if not path.poses: return 0.0
        robot_x, robot_y = odom.pose.pose.position.x, odom.pose.pose.position.y
        yaw = self.quaternion_to_yaw(odom.pose.pose.orientation)

        closest = None
        for p in path.poses:
            dist = math.hypot(p.pose.position.x - robot_x, p.pose.position.y - robot_y)
            if closest is None or dist < closest[0]:
                closest = (dist, p.pose.position.x, p.pose.position.y)

        if closest is None: return 0.0
        target_yaw = math.atan2(closest[2] - robot_y, closest[1] - robot_x)
        return self.normalize_angle(target_yaw - yaw)

    def quaternion_to_yaw(self, orientation) -> float:
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def normalize_angle(self, angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def _check_collision(self) -> int:
        if self.latest_costmap is None or self.latest_odom is None:
            return 0
        try:
            robot_x = self.latest_odom.pose.pose.position.x
            robot_y = self.latest_odom.pose.pose.position.y
            info = self.latest_costmap.info
            ox = info.origin.position.x
            oy = info.origin.position.y
            res = info.resolution
            w = info.width
            h = info.height
            grid = list(self.latest_costmap.data)

            cx = int((robot_x - ox) / res)
            cy = int((robot_y - oy) / res)
            r_cells = int(math.ceil(self.robot_radius_m / res))
            for ry in range(cy - r_cells, cy + r_cells + 1):
                for rx in range(cx - r_cells, cx + r_cells + 1):
                    if rx < 0 or ry < 0 or rx >= w or ry >= h:
                        continue
                    v = grid[ry * w + rx]
                    if v >= 50:
                        return 1
        except Exception:
            return 0
        return 0

    def _min_clearance(self) -> float:
        if self.latest_costmap is None or self.latest_odom is None:
            return 9999.0
        try:
            robot_x = self.latest_odom.pose.pose.position.x
            robot_y = self.latest_odom.pose.pose.position.y
            info = self.latest_costmap.info
            ox = info.origin.position.x
            oy = info.origin.position.y
            res = info.resolution
            w = info.width
            h = info.height
            grid = list(self.latest_costmap.data)
            min_d = 9999.0
            for idx, v in enumerate(grid):
                if v >= 50:
                    ry = idx // w
                    rx = idx % w
                    cell_x = ox + (rx + 0.5) * res
                    cell_y = oy + (ry + 0.5) * res
                    d = math.hypot(cell_x - robot_x, cell_y - robot_y)
                    if d < min_d:
                        min_d = d
            return min_d
        except Exception:
            return 9999.0

    def _save_plot(self) -> None:
        if plt is None or not os.path.exists(self.csv_output_path): return
        try:
            with open(self.csv_output_path, 'r') as f:
                data = list(csv.DictReader(f))

            if not data: return

            times = [float(r['timestamp_s']) for r in data]
            cmd_v = [float(r['cmd_linear_x']) for r in data]
            act_v = [float(r['actual_linear_x']) for r in data]
            jerk = [float(r['steering_jerk_deg_s2']) for r in data]

            plt.figure(figsize=(10, 6))
            
            plt.subplot(2, 1, 1)
            plt.plot(times, cmd_v, label='Commanded Velocity', color='blue', alpha=0.8)
            plt.plot(times, act_v, label='Actual Velocity (Odom)', color='orange', linestyle='--')
            plt.axhline(self.velocity_limit_m_s, color='red', linestyle=':', label='Velocity Cap')
            plt.ylabel('Linear Velocity (m/s)')
            plt.legend(loc='upper right')
            plt.grid(True)

            plt.subplot(2, 1, 2)
            plt.plot(times, jerk, label='Actuator Jerk Profile', color='purple', alpha=0.7)
            plt.axhline(self.max_allowed_jerk_deg_s2, color='orange', linestyle='--', label='Warning Limit')
            plt.axhline(self.instability_jerk_threshold, color='red', linestyle=':', label='Instability Limit')
            plt.ylabel('Jerk (deg/s²)')
            plt.xlabel('Session Time (s)')
            plt.legend(loc='upper right')
            plt.grid(True)

            plt.tight_layout()
            plt.savefig(self.plot_output_path)
            plt.close()
        except Exception as e:
            print(f"[control_monitor] Plot error: {e}", flush=True)

    def _build_per_case_rows_html(self, rows: list) -> str:
        cases_in_order = []
        rows_by_case = {}
        for r in rows:
            cid = r.get('case_id', 'UNKNOWN') or 'UNKNOWN'
            if cid not in rows_by_case:
                rows_by_case[cid] = []
                cases_in_order.append(cid)
            rows_by_case[cid].append(r)

        rows_html = ""
        for cid in cases_in_order:
            case_rows = rows_by_case[cid]
            jerks = [abs(float(r['steering_jerk_deg_s2'])) for r in case_rows]
            lat_errors = [abs(float(r['lat_error_m'])) for r in case_rows]
            overshoots = [float(r['overshoot_m']) for r in case_rows]
            lin_viol = sum(int(r['linear_violation']) for r in case_rows)
            ang_viol = sum(int(r['angular_violation']) for r in case_rows)

            avg_jerk_c = sum(jerks) / len(jerks) if jerks else 0.0
            max_jerk_c = max(jerks) if jerks else 0.0
            avg_lat_c = sum(lat_errors) / len(lat_errors) if lat_errors else 0.0
            max_lat_c = max(lat_errors) if lat_errors else 0.0
            avg_over_c = sum(overshoots) / len(overshoots) if overshoots else 0.0
            max_over_c = max(overshoots) if overshoots else 0.0

            good_samples_c = sum(1 for le in lat_errors if le <= self.similarity_threshold_m)
            similarity_c = good_samples_c / len(case_rows) if case_rows else 0.0

            case_pass = (
                avg_jerk_c <= self.max_allowed_jerk_deg_s2
                and (lin_viol + ang_viol) == 0
                and max_lat_c <= 0.20
            )
            status_color = '#0B0B0B' if case_pass else '#D62828'
            status_text = 'PASS' if case_pass else 'FAIL'

            rows_html += f"""
                  <tr>
                    <td><strong>{cid}</strong></td>
                    <td>{similarity_c * 100:.1f}%</td>
                    <td>{avg_lat_c:.4f}</td>
                    <td>{max_lat_c:.4f}</td>
                    <td>{avg_over_c:.4f}</td>
                    <td>{max_over_c:.4f}</td>
                    <td>{avg_jerk_c:.1f}</td>
                    <td>{max_jerk_c:.1f}</td>
                    <td><span style="display:inline-block; padding:4px 12px; border-radius:3px; font-weight:bold; letter-spacing:0.5px; font-size:12px; background-color:{status_color}; color:#ffffff;">{status_text}</span></td>
                  </tr>"""
        return rows_html

    def _generate_html_report(self) -> None:
        report_path = self.csv_output_path.replace('.csv', '_report.html')
        if not os.path.exists(self.csv_output_path): return

        with open(self.csv_output_path, 'r') as f:
            rows = list(csv.DictReader(f))

        if not rows: return

        # Compute data lists
        jerks = [abs(float(r['steering_jerk_deg_s2'])) for r in rows]
        cmd_v_speeds = [abs(float(r['cmd_linear_x'])) for r in rows]
        act_v_speeds = [abs(float(r['actual_linear_x'])) for r in rows]
        lat_errors = [abs(float(r['lat_error_m'])) for r in rows]
        pos_errors = [float(r['pos_error_m']) for r in rows]
        overshoots = [float(r['overshoot_m']) for r in rows]
        execution_lags = [abs(float(r['cmd_linear_x']) - float(r['actual_linear_x'])) for r in rows]

        # Metric Statistics Calculations
        avg_jerk = sum(jerks) / len(jerks)
        max_jerk = max(jerks)
        avg_lag = sum(execution_lags) / len(execution_lags)

        min_lat, avg_lat, max_lat = min(lat_errors), sum(lat_errors)/len(lat_errors), max(lat_errors)
        min_pos, avg_pos, max_pos = min(pos_errors), sum(pos_errors)/len(pos_errors), max(pos_errors)
        min_over, avg_over, max_over = min(overshoots), sum(overshoots)/len(overshoots), max(overshoots)
        min_v, avg_v, max_v = min(cmd_v_speeds), sum(cmd_v_speeds)/len(cmd_v_speeds), max(cmd_v_speeds)

        final_similarity = float(rows[-1]['similarity_ratio'])

        # General System State Rules
        jerk_healthy = avg_jerk <= self.max_allowed_jerk_deg_s2
        bounds_healthy = (self.linear_vel_violations + self.angular_vel_violations) == 0
        lag_healthy = avg_lag < 0.15 
        tracking_healthy = max_lat <= 0.20

        global_pass = jerk_healthy and bounds_healthy and lag_healthy and tracking_healthy

        per_case_rows_html = self._build_per_case_rows_html(rows)

        plot_base64 = ""
        if os.path.exists(self.plot_output_path):
            with open(self.plot_output_path, "rb") as img_file:
                plot_base64 = base64.b64encode(img_file.read()).decode('utf-8')

        csv_uri = pathlib.Path(self.csv_output_path).resolve().as_uri() if os.path.exists(self.csv_output_path) else ""
        csv_filename = os.path.basename(self.csv_output_path)

        html_content = f"""
        <!DOCTYPE html>
        <html>
          <head>
            <meta charset="utf-8">
            <title>ASU ROAR — Controller Diagnostics Report</title>
            <style>
              body {{ font-family: 'Segoe UI', Tahoma, Arial, sans-serif; margin: 0; background-color: #f2f2f2; color: #1a1a1a; }}
              .page {{ max-width: 1100px; margin: 0 auto; padding: 0 30px 30px 30px; }}
              .header-bar {{ background: #0B0B0B; padding: 22px 30px; margin-bottom: 30px; border-bottom: 6px solid #D62828; }}
              .header-bar h1 {{ color: #ffffff; margin: 0; font-weight: 800; letter-spacing: 2px; text-transform: uppercase; font-size: 26px; }}
              .header-bar .subtitle {{ color: #cfcfcf; margin-top: 4px; font-size: 15px; letter-spacing: 0.5px; }}
              .status-banner {{ padding: 20px; border-radius: 4px; text-align: center; font-size: 22px; font-weight: bold; margin-bottom: 25px; letter-spacing: 0.5px; box-shadow: 0 2px 4px rgba(0,0,0,0.08); }}
              .status-banner.pass {{ background-color: #0B0B0B; color: #ffffff; border: 2px solid #ffffff; }}
              .status-banner.fail {{ background-color: #0B0B0B; color: #ffffff; border: 2px solid #D62828; }}
              .card {{ background: white; padding: 20px; border-radius: 4px; border: 1px solid #e2e2e2; border-top: 4px solid #0B0B0B; margin-bottom: 25px; box-shadow: 0 4px 6px rgba(0,0,0,0.03); }}
              .grid {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 15px; margin-bottom: 10px; }}
              .metric-card {{ background: #fafafa; padding: 15px; border-radius: 4px; border: 1px solid #ececec; border-left: 3px solid #D62828; text-align: center; }}
              .metric-card h4 {{ margin: 0 0 8px 0; color: #4a4a4a; font-size: 11px; text-transform: uppercase; letter-spacing: 0.5px; }}
              .metric-card p {{ margin: 0; font-size: 20px; font-weight: bold; color: #0B0B0B; }}
              .visual-container {{ background: white; padding: 20px; border-radius: 4px; border: 1px solid #e2e2e2; border-top: 4px solid #D62828; box-shadow: 0 4px 6px rgba(0,0,0,0.03); }}
              table {{ width: 100%; border-collapse: collapse; margin-top: 10px; }}
              th, td {{ border: 1px solid #e2e2e2; padding: 12px; text-align: center; font-size: 14px; }}
              th {{ background: #0B0B0B; font-weight: bold; color: #ffffff; text-transform: uppercase; font-size: 12px; letter-spacing: 0.5px; }}
              tr:nth-child(even) td {{ background: #fafafa; }}
              img {{ width: 100%; height: auto; display: block; margin: 0 auto; }}
              .csv-link-row {{ text-align: right; margin-bottom: 20px; }}
              .csv-link {{ display: inline-flex; align-items: center; gap: 6px; background: #ffffff; color: #0B0B0B; border: 1px solid #0B0B0B; padding: 8px 16px; border-radius: 4px; text-decoration: none; font-weight: bold; font-size: 13px; letter-spacing: 0.3px; }}
              .csv-link:hover {{ background: #0B0B0B; color: #ffffff; }}
            </style>
          </head>
          <body>
            <div class="header-bar">
              <h1>ASU <span style="color:#D62828;">ROAR</span></h1>
              <div class="subtitle">Continuous Controller Stability &amp; Performance Envelope Evaluation</div>
            </div>
            <div class="page">
            {f'<div class="csv-link-row"><a class="csv-link" href="{csv_uri}" download>&#8595; Download raw CSV ({csv_filename})</a></div>' if csv_uri else ''}

            <div class="status-banner {'pass' if global_pass else 'fail'}">
              OVERALL SYSTEM EVALUATION: {'PASSED STABILITY & TRACKING AUDIT' if global_pass else 'CRITICAL SIGNALS DETECTED / ENVELOPE BREACHED'}
            </div>

            <div class="card">
              <h3 style="margin-top:0; border-bottom: 2px solid #edf2f7; padding-bottom:6px; color:#0B0B0B;">System Summary Diagnostics</h3>
              <div class="grid">
                <div class="metric-card">
                  <h4>Global Path Similarity</h4>
                  <p style="color: #0B0B0B;">{final_similarity * 100:.1f}%</p>
                </div>
                <div class="metric-card">
                  <h4>Signal Smoothness (Avg Jerk)</h4>
                  <p style="color: {'#0B0B0B' if jerk_healthy else '#D62828'}">{avg_jerk:.1f} deg/s²</p>
                </div>
                <div class="metric-card">
                  <h4>Kinematic Envelope Breaches</h4>
                  <p style="color: {'#0B0B0B' if bounds_healthy else '#D62828'}">{self.linear_vel_violations + self.angular_vel_violations}</p>
                </div>
                <div class="metric-card">
                  <h4>Mean Control-to-Odom Lag</h4>
                  <p style="color: {'#0B0B0B' if lag_healthy else '#D62828'}">{avg_lag:.3f} m/s</p>
                </div>
              </div>
            </div>

            <div class="card">
              <h3 style="margin-top:0; border-bottom: 2px solid #edf2f7; padding-bottom:6px; color:#0B0B0B;">Whole-System Statistical Analysis</h3>
              <table>
                <thead>
                  <tr>
                    <th>Metric Vector</th>
                    <th>Minimum Recorded</th>
                    <th>Mean Average</th>
                    <th>Maximum Peak</th>
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <td><strong>Lateral Error (m)</strong></td>
                    <td>{min_lat:.4f}</td>
                    <td>{avg_lat:.4f}</td>
                    <td style="font-weight:bold; color: {'#333' if max_lat <= 0.20 else '#D62828'}">{max_lat:.4f}</td>
                  </tr>
                  <tr>
                    <td><strong>Absolute Positional Error (m)</strong></td>
                    <td>{min_pos:.4f}</td>
                    <td>{avg_pos:.4f}</td>
                    <td>{max_pos:.4f}</td>
                  </tr>
                  <tr>
                    <td><strong>Rotational Overshoot (m)</strong></td>
                    <td>{min_over:.4f}</td>
                    <td>{avg_over:.4f}</td>
                    <td>{max_over:.4f}</td>
                  </tr>
                  <tr>
                    <td><strong>Target Linear Velocity (m/s)</strong></td>
                    <td>{min_v:.2f}</td>
                    <td>{avg_v:.2f}</td>
                    <td>{max_v:.2f}</td>
                  </tr>
                  <tr>
                    <td><strong>Actuator Jerk Response (deg/s²)</strong></td>
                    <td>-{max_jerk:.1f}</td>
                    <td>{avg_jerk:.1f}</td>
                    <td>{max_jerk:.1f}</td>
                  </tr>
                </tbody>
              </table>
            </div>

            <div class="visual-container">
              <h3 style="margin-top:0; color:#2d3748; border-bottom: 2px solid #e2e8f0; padding-bottom:6px;">Continuous Control Loop Telemetry</h3>
              <img src="data:image/png;base64,{plot_base64}" alt="Telemetry Plots" />
            </div>

            <div class="card">
              <h3 style="margin-top:0; border-bottom: 2px solid #edf2f7; padding-bottom:6px; color:#0B0B0B;">Per-Test Breakdown</h3>
              <table>
                <thead>
                  <tr>
                    <th>Test Case</th>
                    <th>Similarity</th>
                    <th>Avg Lat. Error (m)</th>
                    <th>Max Lat. Error (m)</th>
                    <th>Avg Overshoot (m)</th>
                    <th>Max Overshoot (m)</th>
                    <th>Avg Jerk (deg/s²)</th>
                    <th>Max Jerk (deg/s²)</th>
                    <th>Status</th>
                  </tr>
                </thead>
                <tbody>{per_case_rows_html}
                </tbody>
              </table>
            </div>
            </div>
          </body>
        </html>
        """
        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(html_content)
            print(f"[control_monitor] HTML report generated successfully: {report_path}", flush=True)
            self._open_in_browser(report_path)
        except Exception as e:
            print(f"[control_monitor] Failed report write: {e}", flush=True)

    def _open_in_browser(self, report_path: str) -> None:
        abs_path = os.path.abspath(report_path)
        file_uri = pathlib.Path(abs_path).resolve().as_uri()

        is_wsl = False
        if os.path.exists('/proc/version'):
            try:
                with open('/proc/version', 'r') as f:
                    if 'microsoft' in f.read().lower():
                        is_wsl = True
            except Exception as e:
                print(f"[control_monitor] Failed to determine if in WSL: {e}", flush=True)
        
        if is_wsl:
            if subprocess.run(['which', 'wslview'], capture_output=True).returncode == 0:
                try:
                    subprocess.Popen(['wslview', abs_path])
                    return
                except Exception as e:
                    print(f"[control_monitor] Failed to open with wslview: {e}", flush=True)

            if subprocess.run(['which', 'powershell.exe'], capture_output=True).returncode == 0:
                try:
                    win_path = subprocess.check_output(['wslpath', '-w', abs_path]).decode().strip()
                    subprocess.Popen(['powershell.exe', '-NoProfile', '-Command', f'Start-Process "{win_path}"'])
                    return
                except Exception as e:
                    print(f"[control_monitor] Failed to open with powershell: {e}", flush=True)

            if subprocess.run(['which', 'cmd.exe'], capture_output=True).returncode == 0:
                try:
                    win_path = subprocess.check_output(['wslpath', '-w', abs_path]).decode().strip()
                    subprocess.Popen(['cmd.exe', '/c', 'start', '', win_path])
                    return
                except Exception as e:
                    print(f"[control_monitor] Failed to open with cmd: {e}", flush=True)

        try:
            webbrowser.open(file_uri)
        except Exception as e:
            print(f"[control_monitor] Failed to open browser: {e}", flush=True)

    def destroy_node(self):
        print("=== CONTROL MONITOR DESTROY_NODE CALLED ===", flush=True)
        if getattr(self, '_has_destroyed', False):
            return
        self._has_destroyed = True

        if plt is not None:
            try:
                self._save_plot()
            except Exception as e:
                print(f"[control_monitor] Failed saving plot: {e}", flush=True)

        try:
            self._generate_html_report()
        except Exception as e:
            print(f"[control_monitor] Failed generating report: {e}", flush=True)

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerGlobalMonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(f"[control_monitor] Spin interrupted by exception: {e}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()