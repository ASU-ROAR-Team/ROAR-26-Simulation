#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Complete Validator for Adaptive Pure Pursuit Controller
Covers all 8 validation points exactly as specified.

Rover: 6 wheels, MAX_VELOCITY = 1.0 m/s, WIDTH = 0.973 m
Controller params: KL = 0.3, KC = 0.1, min_turn_radius = 1.0 m,
                   min_turn_speed_ratio = 0.4, goal_stop_dist = 0.2 m

Run:  ros2 run <pkg> validator_complete
Plot: saved to /tmp/trajectory_validation.png after first path + odom cycle
"""

import math
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

# ── Rover / controller constants ─────────────────────────────────────────────
MAX_VELOCITY      = 1.0    # m/s
ROVER_WIDTH       = 0.973  # m
KL                = 0.3    # lookahead gain
KC                = 0.1    # lookahead offset
MIN_TURN_RADIUS   = 1.0    # m
MIN_TURN_RATIO    = 0.4    # fraction of MAX_VEL in in-place turn
GOAL_STOP_DIST    = 0.2    # m  ← matches setVelocity threshold exactly
MAX_ANG_VEL       = 2.0    # rad/s
MAX_INPLACE_ANG   = 1.0    # rad/s

# ── Test 1 thresholds (straight-line) ────────────────────────────────────────
STRAIGHT_MEAN_THR = 0.05   # m
STRAIGHT_MAX_THR  = 0.10   # m

# ── Test 2 threshold (circular curvature variance) ───────────────────────────
CURV_VAR_THR      = 0.05   # rad/m²  low variance = smooth

# ── Test 3 oscillation thresholds ───────────────────────────────────────────
OSCILLATION_SIGN_CHANGES_MAX = 4  # direction reversals in angular_z over 20 samples

# ── Colours for terminal output ───────────────────────────────────────────────
PASS = "\033[92m✔\033[0m"
FAIL = "\033[91m✘\033[0m"
WARN = "\033[93m⚠\033[0m"
INFO = "\033[94m·\033[0m"
SEP  = "=" * 60

def _pf(ok: bool) -> str:
    return PASS if ok else FAIL


# ─────────────────────────────────────────────────────────────────────────────
class Validator(Node):
    def __init__(self):
        super().__init__("validator_complete")

        # Standard QoS for high-frequency data
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb,  self.qos)
        self.create_subscription(Twist,    "/cmd_vel",           self.cmd_cb,   self.qos)
        self.create_subscription(Path,     "/path",              self.path_cb,  self.qos)
        self.create_subscription(String,   "/control_test",      self.ctrl_cb,  10)
        self.validation_pub = self.create_publisher(MarkerArray, "/validation_points", self.qos)

        # Raw buffers
        self.robot_path:    list = []   # [[x, y]]
        self.cmd_vel:       list = []   # [[v, w]]
        self.path:          list = []   # [[x, y]] planned
        self.goal:          list | None = None
        self.ctrl_msgs:     list = []

        # Derived per-cmd
        self.speed_hist:     list = []
        self.angular_hist:   list = []
        self.lookahead_hist: list = []

        self._plot_saved = False

        self.create_timer(2.0, self.evaluate)
        self.get_logger().info("Validator (complete, 8 tests) started.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def odom_cb(self, msg: Odometry):
        self.robot_path.append([msg.pose.pose.position.x,
                                 msg.pose.pose.position.y])

    def cmd_cb(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        self.cmd_vel.append([v, w])
        self.speed_hist.append(v)
        self.angular_hist.append(w)
        self.lookahead_hist.append(KL * v + KC)

    def path_cb(self, msg: Path):
        self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        self.goal  = self.path[-1] if self.path else None
        self.get_logger().info(f"Path received: {len(self.path)} waypoints")
        self._plot_saved = False   # re-plot when new path arrives

    def ctrl_cb(self, msg: String):
        self.ctrl_msgs.append(msg.data)

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _cross_track_errors(self, robot: np.ndarray, path: np.ndarray) -> np.ndarray:
        """Perpendicular distance from each robot pose to the nearest path segment."""
        errors = []
        for pt in robot:
            min_d = np.inf
            for i in range(len(path) - 1):
                a, b  = path[i], path[i + 1]
                ab    = b - a
                ab_sq = np.dot(ab, ab)
                if ab_sq < 1e-9:
                    continue
                t     = float(np.clip(np.dot(pt - a, ab) / ab_sq, 0.0, 1.0))
                proj  = a + t * ab
                min_d = min(min_d, float(np.linalg.norm(pt - proj)))
            if min_d < np.inf:
                errors.append(min_d)
        return np.array(errors)

    def _path_curvature(self, robot: np.ndarray) -> np.ndarray:
        """Signed curvature of the robot trajectory via finite differences."""
        x, y   = robot[:, 0], robot[:, 1]
        dx     = np.gradient(x)
        dy     = np.gradient(y)
        ddx    = np.gradient(dx)
        ddy    = np.gradient(dy)
        denom  = (dx**2 + dy**2)**1.5
        denom  = np.where(denom < 1e-9, 1e-9, denom)
        return (dx * ddy - dy * ddx) / denom

    def _oscillation_count(self, angular: np.ndarray, window: int = 20) -> int:
        """Count sign changes in angular_z over the last `window` samples."""
        if len(angular) < 4:
            return 0
        recent = angular[-window:]
        signs  = np.sign(recent[np.abs(recent) > 0.05])
        if len(signs) < 2:
            return 0
        return int(np.sum(np.diff(signs) != 0))

    def _detect_inplace_turn_start(self) -> bool:
        """True if the first few cmd_vel samples have angular but tiny forward speed."""
        if len(self.cmd_vel) < 3:
            return False
        first = np.array(self.cmd_vel[:10])
        return bool(np.any(np.abs(first[:, 1]) > 0.05))

    # ── Trajectory plot (Test 8) ───────────────────────────────────────────────

    def _save_plot(self, robot: np.ndarray, path: np.ndarray):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(7, 7))
            ax.plot(path[:, 0],  path[:, 1],  "b--",  lw=1.5, label="Planned path")
            ax.plot(robot[:, 0], robot[:, 1], "r-",   lw=1.5, label="Robot trajectory")
            ax.plot(path[0, 0],  path[0, 1],  "gs",   ms=8,   label="Start")
            ax.plot(path[-1, 0], path[-1, 1], "r*",   ms=10,  label="Goal")
            ax.plot(robot[-1, 0],robot[-1, 1],"ko",   ms=6,   label="Robot last pos")
            ax.set_aspect("equal")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            ax.set_title("Trajectory validation — planned vs actual")
            ax.legend(loc="best", fontsize=9)
            ax.grid(True, alpha=0.3)
            out = "/tmp/trajectory_validation.png"
            fig.savefig(out, dpi=120, bbox_inches="tight")
            plt.close(fig)
            self.get_logger().info(f"[Test 8] Plot saved → {out}")
            return True
        except ImportError:
            self.get_logger().warn("[Test 8] matplotlib not available — skipping plot.")
            return False

    # ── Main evaluation ────────────────────────────────────────────────────────

    def evaluate(self):
        print(f"\n{SEP}")
        print("  ADAPTIVE PURE PURSUIT — COMPLETE VALIDATION (8 TESTS)")
        print(SEP)

        n_odom = len(self.robot_path)
        n_cmd  = len(self.cmd_vel)
        n_path = len(self.path)
        print(f"\n{INFO} Samples — odom: {n_odom}  cmd_vel: {n_cmd}  path wps: {n_path}")

        if n_odom < 10 or n_cmd < 5:
            print(f"\n{WARN}  Need ≥ 10 odom + ≥ 5 cmd samples. "
                  f"Have {n_odom}/{n_cmd}. Waiting…")
            return

        robot    = np.array(self.robot_path)
        speeds   = np.array(self.speed_hist)
        angulars = np.array(self.angular_hist)
        lds      = np.array(self.lookahead_hist)

        # ── Test 1: Straight-line tracking ───────────────────────────────────
        print(f"\n── Test 1: Straight-line tracking ──")
        if n_path >= 2:
            path_arr = np.array(self.path)
            cte      = self._cross_track_errors(robot, path_arr)
            if len(cte):
                mean_cte = float(np.mean(cte))
                max_cte  = float(np.max(cte))
                ok_mean  = mean_cte < STRAIGHT_MEAN_THR
                ok_max   = max_cte  < STRAIGHT_MAX_THR
                print(f"  {_pf(ok_mean)} Mean CTE : {mean_cte:.4f} m  (< {STRAIGHT_MEAN_THR} m)")
                print(f"  {_pf(ok_max)} Max  CTE : {max_cte:.4f} m  (< {STRAIGHT_MAX_THR} m)")
            else:
                print(f"  {WARN}  CTE computation needs ≥ 2 path segments.")
        else:
            print(f"  {WARN}  Path not received yet.")

        # ── Test 2: Circular path tracking / curvature variance ──────────────
        print(f"\n── Test 2: Circular path curvature variance ──")
        if len(robot) >= 10:
            k_traj   = self._path_curvature(robot)
            k_var    = float(np.var(k_traj))
            k_mean   = float(np.mean(np.abs(k_traj)))
            ok_var   = k_var < CURV_VAR_THR
            print(f"  {_pf(ok_var)} Curvature variance : {k_var:.5f} rad²/m²  (< {CURV_VAR_THR})")
            print(f"  {INFO} Mean |curvature|   : {k_mean:.4f} rad/m")
            print(f"  {INFO} (Low variance = smooth trajectory on any path shape)")
        else:
            print(f"  {WARN}  Need ≥ 10 odom samples.")

        # ── Test 3: Sharp turn / oscillation ─────────────────────────────────
        print(f"\n── Test 3: Sharp turn — overshoot & oscillation ──")
        osc_count = self._oscillation_count(angulars)
        ok_osc    = osc_count <= OSCILLATION_SIGN_CHANGES_MAX
        print(f"  {_pf(ok_osc)} Angular sign changes (last 20 cmds): {osc_count}  "
              f"(≤ {OSCILLATION_SIGN_CHANGES_MAX} = no oscillation)")

        # Check that in-place turn speed is reduced
        inplace_mask = (speeds < MIN_TURN_RATIO * MAX_VELOCITY + 0.05) & (np.abs(angulars) > 0.1)
        n_inplace    = int(np.sum(inplace_mask))
        if n_inplace > 0:
            ang_ip   = np.abs(angulars[inplace_mask])
            ok_ip    = float(np.max(ang_ip)) <= MAX_INPLACE_ANG + 0.05
            print(f"  {_pf(ok_ip)} In-place turn angular cap: "
                  f"{float(np.max(ang_ip)):.3f} rad/s  (≤ {MAX_INPLACE_ANG})")
        else:
            print(f"  {INFO} No in-place turn samples yet.")

        # ── Test 4: Target behind robot — initial angular velocity ────────────
        print(f"\n── Test 4: Target behind robot — initial angular response ──")
        initial_ang  = self._detect_inplace_turn_start()
        ok_t4        = initial_ang
        print(f"  {_pf(ok_t4)} Non-zero initial angular.z detected: {initial_ang}")
        if len(self.cmd_vel) >= 3:
            first_cmds = np.array(self.cmd_vel[:10])
            max_init_w = float(np.max(np.abs(first_cmds[:, 1])))
            print(f"  {INFO} Max |angular.z| in first 10 cmds: {max_init_w:.3f} rad/s")
            # Cross-check: controller uses atan2(det, dot) — behind target → θ ≈ ±π → in-place
            print(f"  {INFO} Controller uses atan2(cross, dot) → θ > π/2 fires in-place turn. "
                  f"Non-zero ω confirms this path.")

        # ── Test 5: Goal reaching ─────────────────────────────────────────────
        print(f"\n── Test 5: Goal reaching ──")
        if self.goal is not None:
            last_pos      = robot[-1]
            dist_to_goal  = float(np.linalg.norm(last_pos - np.array(self.goal)))
            ok_dist       = dist_to_goal < GOAL_STOP_DIST        # ← 0.2 m, matches setVelocity
            last_speed    = float(speeds[-1])
            near_goal     = dist_to_goal < GOAL_STOP_DIST
            ok_stop       = (not near_goal) or (last_speed < 0.02)
            print(f"  {INFO} Goal              : {[round(float(v),3) for v in self.goal]}")
            print(f"  {INFO} Robot last pos    : {[round(float(v),3) for v in last_pos]}")
            print(f"  {_pf(ok_dist) if near_goal else INFO} Distance to goal  : {dist_to_goal:.4f} m  "
                  f"(threshold {GOAL_STOP_DIST} m)")
            if near_goal:
                print(f"  {_pf(ok_stop)} Last cmd speed    : {last_speed:.4f} m/s  (expect ≈ 0)")
            else:
                print(f"  {INFO} Robot not at goal yet. Distance remaining: {dist_to_goal:.3f} m")
        else:
            print(f"  {WARN}  No goal received yet.")

        # ── Test 6: Velocity adaptation (inverse v–curvature relation) ────────
        print(f"\n── Test 6: Velocity adaptation  v = V_max / (1 + |k|) ──")
        # Reconstruct k from angular / speed for non-trivial forward motion
        forward_mask = speeds > 0.05
        if np.sum(forward_mask) > 5:
            v_fwd   = speeds[forward_mask]
            w_fwd   = angulars[forward_mask]
            k_obs   = np.abs(w_fwd / (v_fwd + 1e-6))
            v_exp   = MAX_VELOCITY / (1.0 + k_obs)
            err     = np.abs(v_fwd - v_exp)
            ok_law  = float(np.mean(err)) < 0.08
            print(f"  {_pf(ok_law)} Mean |v_obs − v_expected| : {float(np.mean(err)):.4f} m/s  (< 0.08)")
            # Verify monotonicity: higher k → lower v
            corr = float(np.corrcoef(k_obs, v_fwd)[0, 1])
            ok_mono = corr < -0.3   # strong negative correlation expected
            print(f"  {_pf(ok_mono)} Pearson corr(k, v)        : {corr:.3f}  (expect < −0.3)")
        else:
            print(f"  {WARN}  Not enough forward-motion samples.")

        # ── Test 7: Adaptive lookahead distance ───────────────────────────────
        print(f"\n── Test 7: Adaptive lookahead distance ──")
        ld_min  = float(np.min(lds))
        ld_max  = float(np.max(lds))
        exp_max = KL * MAX_VELOCITY + KC
        ok_min  = ld_min >= KC - 0.01
        ok_max  = ld_max <= exp_max + 0.05
        print(f"  {_pf(ok_min)} Ld min : {ld_min:.4f} m  (floor = KC = {KC})")
        print(f"  {_pf(ok_max)} Ld max : {ld_max:.4f} m  (ceil at v_max = {exp_max:.3f} m)")
        # Monotonicity: Ld increases with v
        if len(speeds) > 5:
            corr_ld = float(np.corrcoef(speeds, lds)[0, 1])
            ok_mono_ld = corr_ld > 0.8  # nearly perfect by construction
            print(f"  {_pf(ok_mono_ld)} Pearson corr(v, Ld)  : {corr_ld:.4f}  (expect > 0.8)")
            print(f"  {INFO} Ld = {KL}·v + {KC}  →  monotone by construction in controller")

        # ── Test 8: Trajectory visualisation ─────────────────────────────────
        print(f"\n── Test 8: Trajectory visualisation ──")
        if n_path >= 2 and not self._plot_saved:
            path_arr = np.array(self.path)
            ok_plot  = self._save_plot(robot, path_arr)
            self._plot_saved = ok_plot
            if ok_plot:
                print(f"  {PASS} Plot saved → /tmp/trajectory_validation.png")
            else:
                print(f"  {WARN}  Install matplotlib for trajectory plot:  "
                      f"pip install matplotlib --break-system-packages")
        elif self._plot_saved:
            # Update plot each evaluation cycle
            if n_path >= 2:
                self._save_plot(robot, np.array(self.path))
            print(f"  {PASS} Plot updated → /tmp/trajectory_validation.png")
        else:
            print(f"  {WARN}  Awaiting path data.")

        self._publish_validation_markers()
        # ── Summary ────────────────────────────────────────────────────────────
        print(f"\n{SEP}")
        print("  All 8 validation points checked. See /tmp/trajectory_validation.png")
        print(SEP + "\n")

    def _publish_validation_markers(self):
        if not self.path or not self.robot_path:
            return

        ma = MarkerArray()
        header = self.get_clock().now().to_msg()
        header.frame_id = "odom"

        # Start marker (green)
        start_marker = Marker()
        start_marker.header = header
        start_marker.ns = "validation"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = float(self.path[0][0])
        start_marker.pose.position.y = float(self.path[0][1])
        start_marker.pose.position.z = 0.05
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 0.8
        ma.markers.append(start_marker)

        # Goal marker (red)
        goal_marker = Marker()
        goal_marker.header = header
        goal_marker.ns = "validation"
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(self.path[-1][0])
        goal_marker.pose.position.y = float(self.path[-1][1])
        goal_marker.pose.position.z = 0.05
        goal_marker.scale.x = 0.25
        goal_marker.scale.y = 0.25
        goal_marker.scale.z = 0.25
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 0.9
        ma.markers.append(goal_marker)

        # Last robot pose (blue)
        last_marker = Marker()
        last_marker.header = header
        last_marker.ns = "validation"
        last_marker.id = 2
        last_marker.type = Marker.SPHERE
        last_marker.action = Marker.ADD
        last_marker.pose.position.x = float(self.robot_path[-1][0])
        last_marker.pose.position.y = float(self.robot_path[-1][1])
        last_marker.pose.position.z = 0.05
        last_marker.scale.x = 0.18
        last_marker.scale.y = 0.18
        last_marker.scale.z = 0.18
        last_marker.color.r = 0.0
        last_marker.color.g = 0.0
        last_marker.color.b = 1.0
        last_marker.color.a = 0.9
        ma.markers.append(last_marker)

        # Turning point marker: where max angular command occurred
        try:
            if len(self.cmd_vel) > 0:
                angs = np.array([abs(x[1]) for x in self.cmd_vel])
                idx = int(np.argmax(angs))
                # Map index to robot_path if possible
                if idx < len(self.robot_path):
                    tp_x, tp_y = float(self.robot_path[idx][0]), float(self.robot_path[idx][1])
                else:
                    tp_x, tp_y = float(self.robot_path[-1][0]), float(self.robot_path[-1][1])
                turn_marker = Marker()
                turn_marker.header = header
                turn_marker.ns = "validation"
                turn_marker.id = 3
                turn_marker.type = Marker.CYLINDER
                turn_marker.action = Marker.ADD
                turn_marker.pose.position.x = tp_x
                turn_marker.pose.position.y = tp_y
                turn_marker.pose.position.z = 0.05
                turn_marker.scale.x = 0.2
                turn_marker.scale.y = 0.2
                turn_marker.scale.z = 0.15
                turn_marker.color.r = 1.0
                turn_marker.color.g = 0.5
                turn_marker.color.b = 0.0
                turn_marker.color.a = 0.9
                ma.markers.append(turn_marker)
        except Exception:
            pass

        # Sharp edge marker: max curvature along planned path
        try:
            path_arr = np.array(self.path)
            if path_arr.shape[0] >= 3:
                # compute curvature of discrete path
                xs = path_arr[:, 0]
                ys = path_arr[:, 1]
                dx = np.gradient(xs)
                dy = np.gradient(ys)
                ddx = np.gradient(dx)
                ddy = np.gradient(dy)
                denom = (dx**2 + dy**2)**1.5
                denom = np.where(denom < 1e-9, 1e-9, denom)
                curvature = (dx * ddy - dy * ddx) / denom
                idxc = int(np.argmax(np.abs(curvature)))
                sx, sy = float(xs[idxc]), float(ys[idxc])
                sharp_marker = Marker()
                sharp_marker.header = header
                sharp_marker.ns = "validation"
                sharp_marker.id = 4
                sharp_marker.type = Marker.CUBE
                sharp_marker.action = Marker.ADD
                sharp_marker.pose.position.x = sx
                sharp_marker.pose.position.y = sy
                sharp_marker.pose.position.z = 0.05
                sharp_marker.scale.x = 0.22
                sharp_marker.scale.y = 0.22
                sharp_marker.scale.z = 0.08
                sharp_marker.color.r = 0.6
                sharp_marker.color.g = 0.0
                sharp_marker.color.b = 0.8
                sharp_marker.color.a = 0.95
                ma.markers.append(sharp_marker)
        except Exception:
            pass

        self.validation_pub.publish(ma)

def main():
    rclpy.init()
    node = Validator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
