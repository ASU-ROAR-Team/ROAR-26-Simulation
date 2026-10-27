#!/usr/bin/env python3
"""
slam_evaluator.py
=================
Grades the SLAM module (localization + mapping) against the team's
"ERC Marsyard Module-Based Benchmarking Criteria" during a teleoperated run.

What it measures (localization — Mohamed's side)
------------------------------------------------
  Pose Error        avg / max / min distance between /odom estimate and
                    Gazebo ground truth                       target ≤ 0.30 m
  Yaw Error         avg |yaw_est − yaw_gt|                    target ≤ 5°
  Drift Rate        pose-error growth per meter traveled and per minute
  Dropouts          gaps in /odom longer than a threshold     target 0
  ArUco utilization corrections applied / detections received
  ZUPT count        zero-velocity updates fired
  Waypoint stops    at each detected stop near a waypoint:
                      - filter-vs-ground-truth error at the stop (our error)
                      - ground-truth distance to the waypoint  (driver skill)
                      - filter distance to the waypoint        (what the
                        judge would compute from OUR estimate)

What it measures (mapping — teammate's side)
--------------------------------------------
  Map Alignment     for every virtual obstacle in the scenario, distance
                    from its true position to the nearest occupied cell in
                    /active_map/occupancy                     target ≤ 0.20 m
  Map coverage      fraction of scenario obstacles that appear in the map

Final score follows the benchmark formula:
  SLAM = 0.30·PoseAcc + 0.15·YawAcc + 0.20·Drift + 0.20·MapAlign
       + 0.10·Continuity + 0.05·UpdateRate

Sub-score mapping (documented, transparent):
  score = 100                          if metric ≤ 0.5 × target
  score = linear 100 → 0               between 0.5× and 2× target
  score = 0                            if metric ≥ 2 × target

Usage
-----
Runs alongside the sim. Prints a live status line every 2 s. On Ctrl+C
(or ROS shutdown) writes CSV logs + a summary report to
~/slam_eval_<timestamp>/ and prints the final score table.
"""

import csv
import math
import os
import time
from datetime import datetime

import rclpy
import yaml
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from roar_msgs.msg import ArucoDetectionArray


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def sub_score(value: float, target: float) -> float:
    """100 at ≤0.5×target, linear to 0 at ≥2×target."""
    if value <= 0.5 * target:
        return 100.0
    if value >= 2.0 * target:
        return 0.0
    return 100.0 * (2.0 * target - value) / (1.5 * target)


class SlamEvaluator(Node):

    def __init__(self):
        super().__init__("slam_evaluator")

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter("scenario_file", "")
        self.declare_parameter("ground_truth_topic", "/ground_truth/pose")
        self.declare_parameter("pose_index", 11)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("aruco_topic", "/aruco/detections")
        self.declare_parameter("diagnostics_topic", "/roar/ieskf_diagnostics")
        self.declare_parameter("occupancy_topic", "/active_map/occupancy")
        self.declare_parameter("output_dir", "")

        scenario_file = self.get_parameter("scenario_file").value
        self._gt_topic = self.get_parameter("ground_truth_topic").value
        self._pose_index = self.get_parameter("pose_index").value
        odom_topic = self.get_parameter("odom_topic").value
        aruco_topic = self.get_parameter("aruco_topic").value
        diag_topic = self.get_parameter("diagnostics_topic").value
        occ_topic = self.get_parameter("occupancy_topic").value

        out_dir = self.get_parameter("output_dir").value
        if not out_dir:
            out_dir = os.path.expanduser(
                "~/slam_eval_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        self._out_dir = out_dir

        # ── Scenario ───────────────────────────────────────────────────────────
        self._waypoints = []
        self._obstacles = []
        self._targets = {
            "pose_error_avg": 0.30,
            "yaw_error_avg": 5.0,
            "map_alignment": 0.20,
            "localization_dropout_max_gap": 0.5,
        }
        self._stop_speed = 0.03
        self._stop_duration = 2.0
        self._capture_radius = 2.5
        self._start_xy = (0.0, 0.0)

        if scenario_file:
            with open(scenario_file, "r") as f:
                data = yaml.safe_load(f)
            sc = data.get("scenario", {})
            self._waypoints = [(w["id"], w["x"], w["y"])
                               for w in sc.get("waypoints", [])]
            self._start_xy = (sc.get("start", {}).get("x", 0.0),
                              sc.get("start", {}).get("y", 0.0))
            self._stop_speed = sc.get("stop_speed_threshold", 0.03)
            self._stop_duration = sc.get("stop_duration", 2.0)
            self._capture_radius = sc.get("waypoint_capture_radius", 2.5)
            self._obstacles = data.get("obstacles", [])
            self._targets.update(data.get("targets", {}))
            self.get_logger().info(
                f"Scenario: {len(self._waypoints)} waypoints, "
                f"{len(self._obstacles)} obstacles")
        else:
            self.get_logger().warn("No scenario_file — waypoint scoring off.")

        # ── State ──────────────────────────────────────────────────────────────
        self._gt_pose = None            # (x, y, yaw)
        self._gt_prev = None            # (x, y, t) for speed + distance
        self._gt_speed = 0.0
        self._distance_traveled = 0.0

        self._samples = []              # (t, dist, err_xy, err_yaw_deg)
        self._err_sum = 0.0
        self._err_max = 0.0
        self._err_min = float("inf")
        self._yaw_sum = 0.0
        self._n_samples = 0

        self._last_odom_walltime = None
        self._dropouts = 0
        self._run_start = time.monotonic()

        # ArUco / diagnostics
        self._aruco_detection_msgs = 0      # non-empty arrays received on topic
        self._aruco_detections_seen = 0     # individual detections on topic
        self._diag = None                   # latest diagnostics array

        # Stop detection
        self._low_speed_since = None
        self._stop_registered = False
        self._stops = []                    # dicts per registered stop
        self._next_wp_idx = 0               # index into waypoints, then start

        # Map
        self._latest_grid = None

        # ── Subscriptions ──────────────────────────────────────────────────────
        self.create_subscription(PoseWithCovarianceStamped, self._gt_topic, self._on_gt, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 50)
        self.create_subscription(ArucoDetectionArray, aruco_topic,
                                 self._on_aruco, 10)
        self.create_subscription(Float64MultiArray, diag_topic,
                                 self._on_diag, 10)
        self.create_subscription(OccupancyGrid, occ_topic, self._on_grid, 1)

        # Live status line
        self.create_timer(2.0, self._print_status)

        self.get_logger().info(
            f"SLAM evaluator running. Output → {self._out_dir}")

    # ── Ground truth ───────────────────────────────────────────────────────────

    def _on_gt(self, msg: PoseWithCovarianceStamped):
        pose = msg.pose.pose
        x, y = pose.position.x, pose.position.y
        yaw = yaw_from_quat(pose.orientation)
        now = time.monotonic()

        if self._gt_prev is not None:
            px, py, pt = self._gt_prev
            dt = now - pt
            if dt > 1e-3:
                step = math.hypot(x - px, y - py)
                self._gt_speed = step / dt
                self._distance_traveled += step
        self._gt_prev = (x, y, now)
        self._gt_pose = (x, y, yaw)

        self._update_stop_detection(x, y, now)

    # ── Stop / waypoint logic ──────────────────────────────────────────────────

    def _expected_target(self):
        """Current expected stop target: waypoints in order, then start."""
        if self._next_wp_idx < len(self._waypoints):
            wid, wx, wy = self._waypoints[self._next_wp_idx]
            return (f"WP{wid}", wx, wy)
        return ("END(start)", self._start_xy[0], self._start_xy[1])

    def _update_stop_detection(self, x, y, now):
        if not self._waypoints:
            return

        if self._gt_speed < self._stop_speed:
            if self._low_speed_since is None:
                self._low_speed_since = now
            elif (now - self._low_speed_since >= self._stop_duration
                  and not self._stop_registered):
                self._register_stop(x, y)
                self._stop_registered = True
        else:
            self._low_speed_since = None
            self._stop_registered = False

    def _register_stop(self, gx, gy):
        label, tx, ty = self._expected_target()
        dist_to_target = math.hypot(gx - tx, gy - ty)

        # Only count stops near the expected target — random pauses on the
        # route are ignored for waypoint scoring (ZUPT still fires on them).
        if dist_to_target > self._capture_radius:
            return

        # Filter estimate at the moment of the stop
        est = self._samples[-1] if self._samples else None
        filter_err = est[2] if est else float("nan")

        # Filter's distance to the waypoint (what a judge reading OUR
        # estimate would compute)
        fx, fy = self._last_filter_xy if hasattr(self, "_last_filter_xy") \
            else (float("nan"), float("nan"))
        filter_dist_to_target = math.hypot(fx - tx, fy - ty)

        self._stops.append({
            "target": label,
            "gt_dist_to_target": dist_to_target,
            "filter_dist_to_target": filter_dist_to_target,
            "filter_vs_gt_error": filter_err,
            "t": time.monotonic() - self._run_start,
        })
        self.get_logger().info(
            f"STOP at {label}: GT dist={dist_to_target:.3f} m | "
            f"filter dist={filter_dist_to_target:.3f} m | "
            f"filter err={filter_err:.3f} m")
        self._next_wp_idx += 1

    # ── Filter odometry ────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        now_wall = time.monotonic()

        # Dropout detection
        if self._last_odom_walltime is not None:
            gap = now_wall - self._last_odom_walltime
            if gap > self._targets["localization_dropout_max_gap"]:
                self._dropouts += 1
                self.get_logger().warn(
                    f"LOCALIZATION DROPOUT: {gap:.2f} s without /odom")
        self._last_odom_walltime = now_wall

        if self._gt_pose is None:
            return

        fx = msg.pose.pose.position.x
        fy = msg.pose.pose.position.y
        fyaw = yaw_from_quat(msg.pose.pose.orientation)
        self._last_filter_xy = (fx, fy)

        gx, gy, gyaw = self._gt_pose
        err = math.hypot(fx - gx, fy - gy)
        yaw_err = abs(wrap_deg(math.degrees(fyaw - gyaw)))

        self._n_samples += 1
        self._err_sum += err
        self._yaw_sum += yaw_err
        self._err_max = max(self._err_max, err)
        self._err_min = min(self._err_min, err)

        # Keep 1-in-10 samples for the CSV / drift fit (odom is ~100 Hz)
        if self._n_samples % 10 == 0:
            self._samples.append((
                now_wall - self._run_start,
                self._distance_traveled,
                err,
                yaw_err,
            ))

    # ── ArUco / diagnostics / map ──────────────────────────────────────────────

    def _on_aruco(self, msg: ArucoDetectionArray):
        if msg.detections:
            self._aruco_detection_msgs += 1
            self._aruco_detections_seen += len(msg.detections)

    def _on_diag(self, msg: Float64MultiArray):
        if len(msg.data) >= 6:
            self._diag = list(msg.data)

    def _on_grid(self, msg: OccupancyGrid):
        self._latest_grid = msg

    # ── Map alignment metric ───────────────────────────────────────────────────

    def _map_alignment(self):
        """
        For each scenario obstacle: distance from its true center to the
        nearest occupied cell center. Returns (avg_error, coverage) or
        (None, 0.0) if no grid / no obstacles.
        """
        if self._latest_grid is None or not self._obstacles:
            return None, 0.0

        g = self._latest_grid
        res = g.info.resolution
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        w = g.info.width

        occupied = []
        for i, v in enumerate(g.data):
            if v > 50:
                row = i // w
                col = i % w
                cx = ox + (col + 0.5) * res
                cy = oy + (row + 0.5) * res
                occupied.append((cx, cy))

        if not occupied:
            return None, 0.0

        errors = []
        found = 0
        for obs in self._obstacles:
            best = min(math.hypot(obs["x"] - cx, obs["y"] - cy)
                       for cx, cy in occupied)
            # An obstacle "appears in the map" if an occupied cell is within
            # its own footprint radius + 3 cells of slack.
            slack = max(obs["dim_x"], obs["dim_y"]) / 2.0 + 3 * res
            if best <= slack:
                found += 1
                errors.append(best)

        coverage = found / len(self._obstacles)
        avg_err = sum(errors) / len(errors) if errors else None
        return avg_err, coverage

    # ── Drift rate ─────────────────────────────────────────────────────────────

    def _drift_rate(self):
        """
        Least-squares slope of pose error vs distance traveled [m error / m]
        and vs time [m error / min]. Returns (per_meter, per_minute).
        """
        pts = [(d, e) for (_, d, e, _) in self._samples if d > 0.5]
        per_m = self._lsq_slope(pts)
        pts_t = [(t / 60.0, e) for (t, _, e, _) in self._samples]
        per_min = self._lsq_slope(pts_t)
        return per_m, per_min

    @staticmethod
    def _lsq_slope(pts):
        n = len(pts)
        if n < 10:
            return None
        sx = sum(p[0] for p in pts)
        sy = sum(p[1] for p in pts)
        sxx = sum(p[0] * p[0] for p in pts)
        sxy = sum(p[0] * p[1] for p in pts)
        denom = n * sxx - sx * sx
        if abs(denom) < 1e-12:
            return None
        return (n * sxy - sx * sy) / denom

    # ── Live status ────────────────────────────────────────────────────────────

    def _print_status(self):
        if self._n_samples == 0:
            self.get_logger().info("Waiting for /odom + ground truth ...")
            return
        avg = self._err_sum / self._n_samples
        yaw = self._yaw_sum / self._n_samples
        aruco_applied = int(self._diag[1]) if self._diag else 0
        zupt = int(self._diag[2]) if self._diag else 0
        label, _, _ = self._expected_target() if self._waypoints \
            else ("—", 0, 0)
        self.get_logger().info(
            f"err avg={avg:.3f} max={self._err_max:.3f} m | "
            f"yaw={yaw:.2f}° | dist={self._distance_traveled:.1f} m | "
            f"aruco={aruco_applied} zupt={zupt} | "
            f"stops={len(self._stops)} next={label}")

    # ── Final report ───────────────────────────────────────────────────────────

    def write_report(self):
        os.makedirs(self._out_dir, exist_ok=True)

        # ── CSV: time series ───────────────────────────────────────────────────
        with open(os.path.join(self._out_dir, "pose_error.csv"), "w",
                  newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(["t_sec", "distance_m", "pose_error_m",
                           "yaw_error_deg"])
            wcsv.writerows(self._samples)

        # ── CSV: waypoint stops ────────────────────────────────────────────────
        with open(os.path.join(self._out_dir, "waypoint_stops.csv"), "w",
                  newline="") as f:
            wcsv = csv.writer(f)
            wcsv.writerow(["target", "gt_dist_to_target_m",
                           "filter_dist_to_target_m", "filter_vs_gt_error_m",
                           "t_sec"])
            for s in self._stops:
                wcsv.writerow([s["target"], f"{s['gt_dist_to_target']:.4f}",
                               f"{s['filter_dist_to_target']:.4f}",
                               f"{s['filter_vs_gt_error']:.4f}",
                               f"{s['t']:.1f}"])

        # ── Metrics ────────────────────────────────────────────────────────────
        n = max(self._n_samples, 1)
        avg_err = self._err_sum / n
        avg_yaw = self._yaw_sum / n
        drift_m, drift_min = self._drift_rate()
        map_err, map_cov = self._map_alignment()

        aruco_received = int(self._diag[0]) if self._diag else 0
        aruco_applied = int(self._diag[1]) if self._diag else 0
        zupt_count = int(self._diag[2]) if self._diag else 0
        aruco_util = (100.0 * aruco_applied / aruco_received
                      if aruco_received else 0.0)

        # ── Sub-scores per the benchmark formula ──────────────────────────────
        t = self._targets
        s_pose = sub_score(avg_err, t["pose_error_avg"])
        s_yaw = sub_score(avg_yaw, t["yaw_error_avg"])
        # Drift: target = keep error growth under 1% of distance
        s_drift = sub_score(drift_m, 0.01) if drift_m is not None else 0.0
        s_map = sub_score(map_err, t["map_alignment"]) \
            if map_err is not None else 0.0
        s_cont = 100.0 if self._dropouts == 0 else max(
            0.0, 100.0 - 25.0 * self._dropouts)
        # Update rate: mapping publishes at 2 Hz — score present/absent
        s_rate = 100.0 if self._latest_grid is not None else 0.0

        slam_score = (0.30 * s_pose + 0.15 * s_yaw + 0.20 * s_drift
                      + 0.20 * s_map + 0.10 * s_cont + 0.05 * s_rate)

        # ── Summary text ───────────────────────────────────────────────────────
        lines = []
        a = lines.append
        a("=" * 66)
        a("SLAM MODULE EVALUATION REPORT")
        a("=" * 66)
        a(f"run duration        : {time.monotonic() - self._run_start:.1f} s")
        a(f"distance traveled   : {self._distance_traveled:.1f} m")
        a(f"odom samples        : {self._n_samples}")
        a("")
        a("LOCALIZATION")
        a(f"  pose error avg    : {avg_err:.3f} m   (target ≤ "
          f"{t['pose_error_avg']:.2f})")
        a(f"  pose error max    : {self._err_max:.3f} m")
        a(f"  pose error min    : {self._err_min:.3f} m")
        a(f"  yaw error avg     : {avg_yaw:.2f}°    (target ≤ "
          f"{t['yaw_error_avg']:.1f})")
        a(f"  drift rate        : "
          + (f"{drift_m*100:.2f} cm/m" if drift_m is not None else "n/a")
          + " | "
          + (f"{drift_min*100:.2f} cm/min" if drift_min is not None else "n/a"))
        a(f"  dropouts          : {self._dropouts}   (target 0)")
        a(f"  aruco received    : {aruco_received}")
        a(f"  aruco applied     : {aruco_applied}  ({aruco_util:.1f}%)")
        a(f"  zupt fired        : {zupt_count}")
        a("")
        a("WAYPOINT STOPS")
        if self._stops:
            for s in self._stops:
                a(f"  {s['target']:<12} gt_dist={s['gt_dist_to_target']:.3f} m"
                  f"  filter_dist={s['filter_dist_to_target']:.3f} m"
                  f"  filter_err={s['filter_vs_gt_error']:.3f} m")
        else:
            a("  (none registered)")
        a("")
        a("MAPPING")
        a(f"  map alignment     : "
          + (f"{map_err:.3f} m   (target ≤ {t['map_alignment']:.2f})"
             if map_err is not None else "n/a — no occupied cells / no grid"))
        a(f"  obstacle coverage : {100.0*map_cov:.0f}% of scenario obstacles")
        a("")
        a("SCORES  (0.30·Pose + 0.15·Yaw + 0.20·Drift + 0.20·Map "
          "+ 0.10·Cont + 0.05·Rate)")
        a(f"  PoseAccuracy      : {s_pose:6.1f}")
        a(f"  YawAccuracy       : {s_yaw:6.1f}")
        a(f"  DriftScore        : {s_drift:6.1f}")
        a(f"  MapAlignment      : {s_map:6.1f}")
        a(f"  Continuity        : {s_cont:6.1f}")
        a(f"  UpdateRate        : {s_rate:6.1f}")
        a(f"  ─────────────────────────")
        a(f"  SLAM SCORE        : {slam_score:6.1f} / 100   "
          f"({'PASS' if slam_score >= 85 else 'FAIL'} — pass ≥ 85)")
        a("=" * 66)

        report = "\n".join(lines)
        with open(os.path.join(self._out_dir, "report.txt"), "w") as f:
            f.write(report + "\n")
        print("\n" + report + "\n")
        print(f"Logs written to: {self._out_dir}")


def main(args=None):
    rclpy.init(args=args)
    node = SlamEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.write_report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
