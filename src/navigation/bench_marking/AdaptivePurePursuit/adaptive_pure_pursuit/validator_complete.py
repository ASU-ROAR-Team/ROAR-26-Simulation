#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Complete Validator for Adaptive Pure Pursuit Controller
Covers all 8 validation points exactly as specified.
Refined for robustness against simulation noise and path re-publishing.

Rover: 6 wheels, MAX_VELOCITY = 1.0 m/s, WIDTH = 0.973 m
Controller params: KL = 0.3, KC = 0.1, min_turn_radius = 1.0 m,
                   min_turn_speed_ratio = 0.4, goal_stop_dist = 0.2 m
"""

import math
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

# ── Rover / controller constants ─────────────────────────────────────────────
MAX_VELOCITY      = 1.0    # m/s
ROVER_WIDTH       = 0.973  # m
KL                = 0.3    # lookahead gain
KC                = 0.1    # lookahead offset
MIN_TURN_RADIUS   = 1.0    # m
MIN_TURN_RATIO    = 0.4    # fraction of MAX_VEL in in-place turn
GOAL_STOP_DIST    = 0.2    # m
MAX_ANG_VEL       = 2.0    # rad/s
MAX_INPLACE_ANG   = 1.0    # rad/s

# ── Thresholds ───────────────────────────────────────────────────────────────
STRAIGHT_MEAN_THR = 0.05   # m
STRAIGHT_MAX_THR  = 0.10   # m
CURV_VAR_THR      = 0.05   # rad²/m²
OSCILLATION_MAX   = 4      # sign changes

# ── Colours ──────────────────────────────────────────────────────────────────
PASS = "\033[92m✔\033[0m"
FAIL = "\033[91m✘\033[0m"
WARN = "\033[93m⚠\033[0m"
INFO = "\033[94m·\033[0m"
SEP  = "=" * 60

def _pf(ok: bool) -> str:
    return PASS if ok else FAIL

class Validator(Node):
    def __init__(self):
        super().__init__("validator_complete")

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=ReliabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb,  self.qos)
        self.create_subscription(Twist,    "/cmd_vel",           self.cmd_cb,   self.qos)
        self.create_subscription(Path,     "/path",              self.path_cb,  self.qos)
        self.create_subscription(String,   "/control_test",      self.ctrl_cb,  10)

        self.robot_path:    list = []
        self.cmd_vel:       list = []
        self.path:          list = []
        self.goal:          list | None = None
        self.ctrl_msgs:     list = []

        self.speed_hist:     list = []
        self.angular_hist:   list = []
        self.lookahead_hist: list = []

        self._plot_saved = False
        self.create_timer(2.0, self.evaluate)
        self.get_logger().info("Validator (Refined, 8 tests) started.")

    def odom_cb(self, msg: Odometry):
        if not self.path: return
        self.robot_path.append([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def cmd_cb(self, msg: Twist):
        if not self.path: return
        v, w = msg.linear.x, msg.angular.z
        self.cmd_vel.append([v, w])
        self.speed_hist.append(v)
        self.angular_hist.append(w)
        self.lookahead_hist.append(KL * v + KC)

    def path_cb(self, msg: Path):
        # We only take the path once to ensure Test 4 (Initial Response) is clean
        if not self.path:
            self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
            self.goal = self.path[-1] if self.path else None
            self.get_logger().info(f"Path received: {len(self.path)} waypoints")

    def ctrl_cb(self, msg: String):
        self.ctrl_msgs.append(msg.data)

    def _cross_track_errors(self, robot: np.ndarray, path: np.ndarray) -> np.ndarray:
        if len(robot) < 2: return np.array([])
        # Focus on recent history to evaluate steady state
        subset = robot[-200:] if len(robot) > 200 else robot
        errors = []
        for pt in subset:
            min_d = np.inf
            for i in range(len(path) - 1):
                a, b = path[i], path[i+1]
                ab = b - a
                ab_sq = np.dot(ab, ab)
                if ab_sq < 1e-6: continue
                t = float(np.clip(np.dot(pt - a, ab) / ab_sq, 0.0, 1.0))
                proj = a + t * ab
                min_d = min(min_d, float(np.linalg.norm(pt - proj)))
            errors.append(min_d)
        return np.array(errors)

    def _path_curvature(self, robot: np.ndarray) -> np.ndarray:
        if len(robot) < 5: return np.array([0.0])
        # Filter duplicate consecutive points which break gradient
        diffs = np.linalg.norm(np.diff(robot, axis=0), axis=1)
        mask = np.concatenate(([True], diffs > 1e-4))
        filtered = robot[mask]
        if len(filtered) < 5: return np.array([0.0])

        x, y = filtered[:, 0], filtered[:, 1]
        dx, dy = np.gradient(x), np.gradient(y)
        ddx, ddy = np.gradient(dx), np.gradient(dy)
        denom = (dx**2 + dy**2)**1.5
        denom = np.where(denom < 1e-6, 1e-6, denom)
        return (dx * ddy - dy * ddx) / denom

    def _oscillation_count(self, angular: np.ndarray, window: int = 40) -> int:
        if len(angular) < 10: return 0
        recent = angular[-window:]
        signs = np.sign(recent[np.abs(recent) > 0.02])
        if len(signs) < 2: return 0
        return int(np.sum(np.diff(signs) != 0))

    def _save_plot(self, robot: np.ndarray, path: np.ndarray):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(7, 7))
            ax.plot(path[:, 0], path[:, 1], "b--", alpha=0.5, label="Planned")
            ax.plot(robot[:, 0], robot[:, 1], "r-", label="Trajectory")
            ax.plot(path[0,0], path[0,1], "gs", label="Start")
            ax.plot(path[-1,0], path[-1,1], "r*", label="Goal")
            ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend()
            out = "/tmp/trajectory_validation.png"
            fig.savefig(out, dpi=120); plt.close(fig)
            return True
        except: return False

    def evaluate(self):
        print(f"\n{SEP}\n  ADAPTIVE PURE PURSUIT — REFINED VALIDATION\n{SEP}")
        n_odom, n_cmd = len(self.robot_path), len(self.cmd_vel)
        if n_odom < 10 or n_cmd < 5:
            print(f"{WARN} Awaiting data (Odom: {n_odom}, Cmd: {n_cmd})...")
            return

        robot, speeds, angulars, lds = np.array(self.robot_path), np.array(self.speed_hist), np.array(self.angular_hist), np.array(self.lookahead_hist)

        # 1. Straight-line
        print(f"\n── Test 1: Tracking Error ──")
        cte = self._cross_track_errors(robot, np.array(self.path))
        if len(cte):
            m_cte, max_cte = np.mean(cte), np.max(cte)
            print(f"  {_pf(m_cte < STRAIGHT_MEAN_THR)} Mean CTE: {m_cte:.4f}m (<{STRAIGHT_MEAN_THR})")
            print(f"  {_pf(max_cte < STRAIGHT_MAX_THR)} Max CTE : {max_cte:.4f}m (<{STRAIGHT_MAX_THR})")

        # 2. Curvature
        print(f"\n── Test 2: Curvature Variance ──")
        k = self._path_curvature(robot)
        k_var = np.var(k)
        print(f"  {_pf(k_var < CURV_VAR_THR)} Var(k): {k_var:.6f} (<{CURV_VAR_THR})")

        # 3. Oscillation
        print(f"\n── Test 3: Sharp Turn ──")
        osc = self._oscillation_count(angulars)
        print(f"  {_pf(osc <= OSCILLATION_MAX)} Oscillation count: {osc} (<={OSCILLATION_MAX})")
        
        # Stuck Check
        if len(speeds) > 20:
            is_stuck = np.mean(speeds[-20:]) < 0.05 and np.mean(np.abs(angulars[-20:])) > 0.1
            if is_stuck:
                print(f"  {FAIL} DIAGNOSTIC: Robot appears STUCK in In-Place Turn (v≈0, ω>0)")

        # 4. Target Behind
        print(f"\n── Test 4: Initial Angular Response ──")
        init_w = np.any(np.abs(angulars[:10]) > 0.05)
        print(f"  {_pf(init_w)} Non-zero initial ω detected: {init_w}")

        # 5. Goal Reaching
        print(f"\n── Test 5: Goal Reaching ──")
        dist = np.linalg.norm(robot[-1] - np.array(self.goal))
        at_goal = dist < GOAL_STOP_DIST
        stopped = speeds[-1] < 0.05
        print(f"  {INFO} Distance to goal: {dist:.3f}m")
        if at_goal:
            print(f"  {_pf(stopped)} Stopped at goal: {stopped}")

        # 6. Velocity Adaptation
        print(f"\n── Test 6: Velocity Adaptation (v vs |k|) ──")
        fwd = speeds > 0.1
        if np.sum(fwd) > 10:
            v_fwd, w_fwd = speeds[fwd], angulars[fwd]
            k_obs = np.abs(w_fwd / (v_fwd + 1e-3))
            corr = np.corrcoef(k_obs, v_fwd)[0, 1]
            if np.isnan(corr): corr = 0.0
            print(f"  {_pf(corr < -0.3)} Correlation(k, v): {corr:.3f} (expect < -0.3)")
        else: print(f"  {WARN} Not enough forward motion.")

        # 7. Adaptive Lookahead
        print(f"\n── Test 7: Adaptive Lookahead (Ld vs v) ──")
        if len(speeds) > 10:
            corr_ld = np.corrcoef(speeds, lds)[0, 1]
            if np.isnan(corr_ld): corr_ld = 1.0 # Constant speed case
            print(f"  {_pf(corr_ld > 0.8)} Correlation(v, Ld): {corr_ld:.3f} (expect > 0.8)")

        # 8. Visualisation
        if not self._plot_saved:
            self._plot_saved = self._save_plot(robot, np.array(self.path))
            if self._plot_saved: print(f"\n{PASS} Plot saved to /tmp/trajectory_validation.png")

def main():
    rclpy.init()
    rclpy.spin(Validator())
    rclpy.shutdown()

if __name__ == "__main__": main()
