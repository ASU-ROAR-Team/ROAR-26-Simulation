#!/usr/bin/env python3
"""
slam_visualizer.py
==================
Live 8-panel dashboard for the ROAR SLAM test.
Extension of Mohamed's original ieskf_visualizer.py — same colour scheme and
2D/3D/error/readout panels, plus four SLAM-specific panels:

  [A] 2D path (filter vs GT) + ArUco markers + waypoints + covariance ellipse
  [B] 3D rover orientation (roll/pitch/yaw)
  [C] Pose error over time (segments coloured by magnitude)
  [D] Text readout: pose, RPY, current error, cumulative counters
  [E] ArUco detections timeline — one row per known ID, dots when detected
  [F] Diagnostics bars — live k_Q, k_V from the WO-FIS / VO-FIS
  [G] Occupancy map — /active_map/occupancy with obstacle ground truth
  [H] Scoreboard — ArUco received/applied, ZUPT fires, drift metrics

Subscribes:
  /odom                                   nav_msgs/Odometry
  /world/marsyard/pose/info               geometry_msgs/PoseArray (GT)
  /aruco/detections                       roar_msgs/ArucoDetectionArray
  /roar/ieskf_diagnostics                 std_msgs/Float64MultiArray
  /active_map/occupancy                   nav_msgs/OccupancyGrid

Reads (optional):
  scenario YAML → waypoint + obstacle overlays
  aruco_map YAML → known marker positions

Usage
-----
  ros2 run roar_slam_testing slam_visualizer                       # defaults
  ros2 run roar_slam_testing slam_visualizer --ros-args \\
      -p scenario_file:=/path/to/scenario_marsyard.yaml \\
      -p aruco_map_file:=/path/to/aruco_map.yaml

Ctrl+C to close.
"""

import math
import threading

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Ellipse, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rclpy
import yaml
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray
from roar_msgs.msg import ArucoDetectionArray


# ═════════════════════════════════════════════════════════════════════════════
# Small helpers (from the original visualizer, unchanged)
# ═════════════════════════════════════════════════════════════════════════════

def quat_to_rpy(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    roll  = math.degrees(math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2*(w*y - z*x)))))
    yaw   = math.degrees(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
    return roll, pitch, yaw


def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    Rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    return Rz @ Ry @ Rx


def draw_rover_box(ax, roll, pitch, yaw):
    ax.cla()
    l, w, h = 1.0, 0.6, 0.35
    verts = np.array([
        [-l/2,-w/2,-h/2],[l/2,-w/2,-h/2],[l/2,w/2,-h/2],[-l/2,w/2,-h/2],
        [-l/2,-w/2, h/2],[l/2,-w/2, h/2],[l/2,w/2, h/2],[-l/2,w/2, h/2],
    ])
    R = rotation_matrix(roll, pitch, yaw)
    v = verts @ R.T
    faces = [
        [v[j] for j in [0,1,2,3]],
        [v[j] for j in [4,5,6,7]],
        [v[j] for j in [0,1,5,4]],
        [v[j] for j in [2,3,7,6]],
        [v[j] for j in [1,2,6,5]],
        [v[j] for j in [0,3,7,4]],
    ]
    colors = ['#1a9fd4','#1a9fd4','#2ecc71','#2ecc71','#e74c3c','#e74c3c']
    for face, color in zip(faces, colors):
        ax.add_collection3d(Poly3DCollection(
            [face], facecolors=color, edgecolors='white',
            linewidths=0.5, alpha=0.85))
    fwd = R @ np.array([l/2 + 0.2, 0, 0])
    ax.quiver(0, 0, 0, fwd[0], fwd[1], fwd[2], color='yellow', linewidth=2)
    ax.set_xlim([-1,1]); ax.set_ylim([-1,1]); ax.set_zlim([-0.5,0.5])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(f'Orientation\nR={roll:+.1f}°  P={pitch:+.1f}°  Y={yaw:+.1f}°',
                 fontsize=9)
    ax.view_init(elev=20, azim=45)
    ax.set_facecolor('#1a1a2e')


def draw_cov_ellipse(ax, x, y, cov_flat, n_std=3.0, **kw):
    """Position covariance ellipse. cov_flat is the row-major 6x6 flattened
    covariance from Odometry.pose.covariance — we use only the xx/xy/yx/yy
    entries at indices 0, 1, 6, 7."""
    cov = np.array([[cov_flat[0], cov_flat[1]],
                    [cov_flat[6], cov_flat[7]]])
    vals, vecs = np.linalg.eigh(cov)
    vals = np.maximum(vals, 1e-9)
    order = vals.argsort()[::-1]
    vals, vecs = vals[order], vecs[:, order]
    w, h = 2 * n_std * np.sqrt(vals)
    angle = math.degrees(math.atan2(vecs[1,0], vecs[0,0]))
    ax.add_patch(Ellipse((x, y), w, h, angle=angle, **kw))


# ═════════════════════════════════════════════════════════════════════════════
# Data node — collects everything the dashboard renders
# ═════════════════════════════════════════════════════════════════════════════

HISTORY_LEN = 1500   # ~50 s at odom's ~30 Hz effective refresh

class DataNode(Node):

    def __init__(self):
        super().__init__('slam_visualizer')

        # ── Parameters ────────────────────────────────────────────────────────
        default_scenario = ''
        default_aruco    = ''
        try:
            share = get_package_share_directory('roar_slam_testing')
            default_scenario = f'{share}/config/scenario_marsyard.yaml'
        except Exception:
            pass
        try:
            share = get_package_share_directory('roar_localization')
            default_aruco = f'{share}/config/aruco_map.yaml'
        except Exception:
            pass

        self.declare_parameter('scenario_file',   default_scenario)
        self.declare_parameter('aruco_map_file',  default_aruco)
        self.declare_parameter('ground_truth_topic', '/ground_truth/pose')
        self.declare_parameter('pose_index', 11)
        self.declare_parameter('odom_topic',  '/odom')
        self.declare_parameter('aruco_topic', '/aruco/detections')
        self.declare_parameter('diag_topic',  '/roar/ieskf_diagnostics')
        self.declare_parameter('occ_topic',   '/active_map/occupancy')

        self._pose_index = self.get_parameter('pose_index').value

        # ── Static overlays from YAML files ───────────────────────────────────
        self.waypoints = []
        self.obstacles = []
        self.start_xy = (0.0, 0.0)
        scen = self.get_parameter('scenario_file').value
        if scen:
            try:
                with open(scen, 'r') as f:
                    data = yaml.safe_load(f)
                sc = data.get('scenario', {})
                self.waypoints = [(w['id'], w['x'], w['y'])
                                  for w in sc.get('waypoints', [])]
                self.start_xy = (sc.get('start', {}).get('x', 0.0),
                                 sc.get('start', {}).get('y', 0.0))
                self.obstacles = data.get('obstacles', [])
                self.get_logger().info(
                    f'Scenario overlay loaded: {len(self.waypoints)} WP, '
                    f'{len(self.obstacles)} obstacles')
            except Exception as exc:
                self.get_logger().warn(f'Scenario overlay skipped: {exc}')

        self.markers = {}   # id -> (x, y, z)
        amap = self.get_parameter('aruco_map_file').value
        if amap:
            try:
                with open(amap, 'r') as f:
                    for m in yaml.safe_load(f).get('markers', []):
                        self.markers[m['id']] = tuple(m['position'])
                self.get_logger().info(
                    f'ArUco overlay loaded: {len(self.markers)} markers')
            except Exception as exc:
                self.get_logger().warn(f'ArUco overlay skipped: {exc}')

        # ── Live state ────────────────────────────────────────────────────────
        self.lock = threading.Lock()

        self.fx = self.fy = self.fz = 0.0
        self.roll = self.pitch = self.yaw = 0.0
        self.cov = [0.0]*36
        self.gx = self.gy = self.gz = 0.0

        self.filter_path = []
        self.gt_path     = []
        self.error_hist  = []
        self.time_hist   = []
        self.cov_hist    = []
        self.t0          = None

        self.aruco_events = []   # (t_rel, id)
        self.diag = [0.0]*6      # [received, applied, zupt, cov_trace, kQ, kV]

        self.occ_grid = None     # (data, info) tuple

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._filter_cb, 50)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('ground_truth_topic').value,
            self._gt_cb, 10)
        self.create_subscription(
            ArucoDetectionArray,
            self.get_parameter('aruco_topic').value,
            self._aruco_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('diag_topic').value,
            self._diag_cb, 10)
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('occ_topic').value,
            self._occ_cb, 1)

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _gt_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        with self.lock:
            self.gx, self.gy, self.gz = p.position.x, p.position.y, p.position.z

    def _filter_cb(self, msg: Odometry):
        with self.lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self.fx, self.fy, self.fz = p.x, p.y, p.z
            self.roll, self.pitch, self.yaw = quat_to_rpy(q)
            self.cov = list(msg.pose.covariance)

            t = self.get_clock().now().nanoseconds * 1e-9
            if self.t0 is None:
                self.t0 = t
            t_rel = t - self.t0

            err = math.hypot(p.x - self.gx, p.y - self.gy)

            self.filter_path.append((p.x, p.y))
            self.gt_path.append((self.gx, self.gy))
            self.cov_hist.append(self.cov[:])
            self.error_hist.append(err)
            self.time_hist.append(t_rel)

            if len(self.filter_path) > HISTORY_LEN:
                self.filter_path.pop(0)
                self.gt_path.pop(0)
                self.cov_hist.pop(0)
                self.error_hist.pop(0)
                self.time_hist.pop(0)

    def _aruco_cb(self, msg: ArucoDetectionArray):
        with self.lock:
            t = self.get_clock().now().nanoseconds * 1e-9
            t_rel = 0.0 if self.t0 is None else (t - self.t0)
            for d in msg.detections:
                self.aruco_events.append((t_rel, d.id))
            # trim
            if len(self.aruco_events) > 5000:
                self.aruco_events = self.aruco_events[-5000:]

    def _diag_cb(self, msg: Float64MultiArray):
        with self.lock:
            if len(msg.data) >= 6:
                self.diag = list(msg.data[:6])

    def _occ_cb(self, msg: OccupancyGrid):
        with self.lock:
            self.occ_grid = (np.array(msg.data, dtype=np.int8), msg.info)


# ═════════════════════════════════════════════════════════════════════════════
# Panel renderers
# ═════════════════════════════════════════════════════════════════════════════

def render_path(ax, node, fp, gp, cov):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2)

    # Static overlays first — waypoints (yellow) + start (white) + markers (magenta)
    if node.start_xy:
        ax.plot(node.start_xy[0], node.start_xy[1], marker='*',
                markersize=14, color='white', zorder=5, label='start/end')
    for wid, wx, wy in node.waypoints:
        ax.plot(wx, wy, marker='X', markersize=12, color='#f1c40f',
                zorder=5)
        ax.annotate(f'WP{wid}', (wx, wy), textcoords='offset points',
                    xytext=(6, 6), color='#f1c40f', fontsize=8)
    for mid, (mx, my, _) in node.markers.items():
        ax.plot(mx, my, marker='s', markersize=8, color='#e056fd', zorder=4)
        ax.annotate(str(mid), (mx, my), textcoords='offset points',
                    xytext=(4, 4), color='#e056fd', fontsize=7)
    for obs in node.obstacles:
        ax.add_patch(Rectangle(
            (obs['x'] - obs['dim_x']/2, obs['y'] - obs['dim_y']/2),
            obs['dim_x'], obs['dim_y'],
            fill=False, edgecolor='#95a5a6', linewidth=1.0, zorder=3))

    # Live paths
    if fp:
        fx = [p[0] for p in fp]; fy = [p[1] for p in fp]
        gx = [p[0] for p in gp]; gy = [p[1] for p in gp]
        ax.plot(fx, fy, color='#3498db', linewidth=1.5, label='IESKF')
        ax.plot(gx, gy, color='#2ecc71', linewidth=1.5, linestyle='--',
                label='Ground truth')
        ax.plot(fx[-1], fy[-1], 'o', color='#3498db', markersize=8)
        ax.plot(gx[-1], gy[-1], 's', color='#2ecc71', markersize=8)
        draw_cov_ellipse(ax, fx[-1], fy[-1], cov, n_std=3,
                         edgecolor='#e74c3c', facecolor='none',
                         linewidth=1.5, alpha=0.8, linestyle='--')

    ax.set_title('2D Path — Filter vs Ground Truth  '
                 '(★ start, ✕ waypoint, ▪ ArUco, ▫ obstacle)',
                 fontsize=10, color='white')
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('Y [m]', fontsize=8)
    ax.legend(fontsize=7, loc='upper left')
    ax.tick_params(labelsize=7)
    ax.set_aspect('equal', adjustable='datalim')


def render_error(ax, err, t):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.2)
    if err:
        err_arr = np.array(err)
        # Coloured segments by magnitude
        for i in range(1, len(t)):
            color = ('#2ecc71' if err_arr[i] < 0.05 else
                     '#f39c12' if err_arr[i] < 0.15 else '#e74c3c')
            ax.plot(t[i-1:i+1], err_arr[i-1:i+1], color=color, linewidth=1.5)
        ax.axhline(0.05, color='#2ecc71', linestyle=':', alpha=0.5, label='5 cm')
        ax.axhline(0.15, color='#f39c12', linestyle=':', alpha=0.5, label='15 cm')
        ax.axhline(0.30, color='#e74c3c', linestyle=':', alpha=0.5,
                   label='0.30 m target')
        mean_err = float(np.mean(err_arr))
        ax.axhline(mean_err, color='white', linestyle='--', alpha=0.6,
                   label=f'Mean {mean_err*100:.1f} cm')
    ax.set_title('Position error vs GT', fontsize=10, color='white')
    ax.set_xlabel('Time [s]', fontsize=8)
    ax.set_ylabel('Error [m]', fontsize=8)
    ax.legend(fontsize=7, loc='upper right')
    ax.tick_params(labelsize=7)


def render_readout(ax, fx, fy, fz, roll, pitch, yaw, gx, gy, err, diag):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    ax.axis('off')

    err_color = ('#2ecc71' if err < 0.05 else
                 '#f39c12' if err < 0.15 else '#e74c3c')
    received, applied, zupt, cov_tr, kQ, kV = diag

    lines = [
        ('IESKF OUTPUT', 'white', 12, True),
        ('', 'white', 8, False),
        (f'x  = {fx:+8.3f} m', '#3498db', 10, False),
        (f'y  = {fy:+8.3f} m', '#3498db', 10, False),
        (f'z  = {fz:+8.3f} m', '#3498db', 10, False),
        (f'R  = {roll:+6.1f}°  P={pitch:+6.1f}°  Y={yaw:+6.1f}°',
         '#9b59b6', 9, False),
        ('', 'white', 8, False),
        ('GROUND TRUTH', 'white', 12, True),
        (f'x  = {gx:+8.3f} m', '#2ecc71', 10, False),
        (f'y  = {gy:+8.3f} m', '#2ecc71', 10, False),
        ('', 'white', 8, False),
        (f'ERROR  = {err*100:6.2f} cm', err_color, 13, True),
        ('', 'white', 8, False),
        ('COUNTERS', 'white', 11, True),
        (f'ArUco received : {int(received):>5}', '#e056fd', 9, False),
        (f'ArUco applied  : {int(applied):>5}',  '#e056fd', 9, False),
        (f'ZUPT fired     : {int(zupt):>5}',     '#f39c12', 9, False),
        (f'cov(p) trace   : {cov_tr:.2e} m²',   '#e74c3c', 9, False),
    ]
    y_pos = 0.98
    for text, color, size, bold in lines:
        ax.text(0.03, y_pos, text, transform=ax.transAxes,
                color=color, fontsize=size,
                fontweight='bold' if bold else 'normal',
                fontfamily='monospace', va='top')
        y_pos -= 0.055


def render_aruco_timeline(ax, node, events, t_now):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    ax.grid(True, alpha=0.15, axis='x')
    ids = sorted(node.markers.keys())
    if not ids:
        ax.text(0.5, 0.5, 'no ArUco map loaded',
                ha='center', va='center', color='white',
                transform=ax.transAxes)
        return
    # y-axis: one row per known marker ID
    id_to_row = {mid: i for i, mid in enumerate(ids)}
    # events within the last 30 s of runtime
    window = 30.0
    xs, ys = [], []
    for t, mid in events:
        if mid in id_to_row and (t_now - t) < window:
            xs.append(t)
            ys.append(id_to_row[mid])
    if xs:
        ax.scatter(xs, ys, s=10, c='#e056fd', alpha=0.7)
    ax.set_yticks(list(id_to_row.values()))
    ax.set_yticklabels([f'id {mid}' for mid in ids], fontsize=7)
    ax.set_xlabel('Time [s]', fontsize=8)
    ax.set_title(f'ArUco detections (last {int(window)} s)',
                 fontsize=10, color='white')
    ax.set_xlim(max(0.0, t_now - window), max(window, t_now))
    ax.tick_params(labelsize=7)


def render_fis_bars(ax, diag):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    _, _, _, _, kQ, kV = diag
    # kQ range ~ [1, 10]  |  kV range ~ [1, 100]
    kQ_norm = min(1.0, (kQ - 1.0) / 9.0)
    kV_norm = min(1.0, (kV - 1.0) / 99.0)
    ax.barh(['k_Q\n(process)', 'k_V\n(observation)'],
            [kQ_norm, kV_norm],
            color=['#f39c12', '#e056fd'], edgecolor='white')
    ax.set_xlim(0, 1)
    ax.set_title(f'FIS scaling — k_Q={kQ:.2f}  k_V={kV:.2f}',
                 fontsize=10, color='white')
    ax.tick_params(labelsize=8)
    ax.grid(True, alpha=0.15, axis='x')


def render_map(ax, node, occ):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    if occ is None:
        ax.text(0.5, 0.5, 'waiting for /active_map/occupancy',
                ha='center', va='center', color='white',
                transform=ax.transAxes)
        ax.set_title('Occupancy map (mapping module)',
                     fontsize=10, color='white')
        return
    data, info = occ
    grid = data.reshape(info.height, info.width)
    # Show only occupied region — the full 1000×1000 is too big/coarse.
    # Crop to the bounding box of occupied cells + a margin.
    occ_ys, occ_xs = np.where(grid > 50)
    if occ_xs.size > 0:
        margin = 30
        x_lo = max(0, occ_xs.min() - margin)
        x_hi = min(info.width, occ_xs.max() + margin)
        y_lo = max(0, occ_ys.min() - margin)
        y_hi = min(info.height, occ_ys.max() + margin)
        sub = grid[y_lo:y_hi, x_lo:x_hi]
        extent = (info.origin.position.x + x_lo * info.resolution,
                  info.origin.position.x + x_hi * info.resolution,
                  info.origin.position.y + y_lo * info.resolution,
                  info.origin.position.y + y_hi * info.resolution)
    else:
        sub = grid; extent = (
            info.origin.position.x,
            info.origin.position.x + info.width  * info.resolution,
            info.origin.position.y,
            info.origin.position.y + info.height * info.resolution)
    ax.imshow(sub, cmap='gray_r', origin='lower', extent=extent, vmin=-1, vmax=100)
    # Overlay virtual obstacles (rectangles) so you can see mapping accuracy
    for obs in node.obstacles:
        ax.add_patch(Rectangle(
            (obs['x'] - obs['dim_x']/2, obs['y'] - obs['dim_y']/2),
            obs['dim_x'], obs['dim_y'],
            fill=False, edgecolor='#f1c40f', linewidth=1.5, zorder=5))
    ax.set_title('Mapping — occupancy grid + true obstacles (yellow)',
                 fontsize=10, color='white')
    ax.tick_params(labelsize=7)
    ax.set_aspect('equal')


def render_scoreboard(ax, err, diag):
    ax.cla()
    ax.set_facecolor('#1a1a2e')
    ax.axis('off')
    if err:
        avg = float(np.mean(err))
        emax = float(np.max(err))
    else:
        avg = 0.0; emax = 0.0
    received, applied, zupt, _, _, _ = diag
    util = 0.0 if received == 0 else 100.0 * applied / received
    # colour by target
    ok = '#2ecc71'; warn = '#f39c12'; bad = '#e74c3c'
    def col(v, t): return ok if v <= 0.5*t else warn if v <= 2.0*t else bad
    lines = [
        ('LOCALIZATION HEALTH', 'white', 12, True),
        ('', 'white', 8, False),
        (f'avg error     {avg*100:6.2f} cm  (target ≤ 30)', col(avg, 0.30), 10, True),
        (f'max error     {emax*100:6.2f} cm', col(emax, 0.60), 10, False),
        ('', 'white', 8, False),
        (f'ArUco util    {util:5.1f}%  ({int(applied)}/{int(received)})',
         '#e056fd', 10, False),
        (f'ZUPT fires    {int(zupt):5d}', '#f39c12', 10, False),
    ]
    y = 0.95
    for text, color, size, bold in lines:
        ax.text(0.03, y, text, transform=ax.transAxes, color=color,
                fontsize=size,
                fontweight='bold' if bold else 'normal',
                fontfamily='monospace', va='top')
        y -= 0.11


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = DataNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.style.use('dark_background')
    fig = plt.figure(figsize=(18, 10), facecolor='#0d0d1a')
    fig.canvas.manager.set_window_title('ROAR — SLAM Live Dashboard')

    # 4x4 grid, 8 panels
    gs = fig.add_gridspec(4, 4, hspace=0.55, wspace=0.35,
                          left=0.05, right=0.98, top=0.95, bottom=0.05)

    ax_path   = fig.add_subplot(gs[0:2, 0:2])                # A
    ax_3d     = fig.add_subplot(gs[0:2, 2],  projection='3d')# B
    ax_txt    = fig.add_subplot(gs[0:2, 3])                  # D
    ax_err    = fig.add_subplot(gs[2,   0:2])                # C
    ax_aruco  = fig.add_subplot(gs[2,   2:4])                # E
    ax_map    = fig.add_subplot(gs[3,   0:2])                # G
    ax_fis    = fig.add_subplot(gs[3,   2])                  # F
    ax_score  = fig.add_subplot(gs[3,   3])                  # H

    def animate(_):
        with node.lock:
            fp   = list(node.filter_path)
            gp   = list(node.gt_path)
            cov  = node.cov_hist[-1] if node.cov_hist else [0.0]*36
            err  = list(node.error_hist)
            t    = list(node.time_hist)
            r, p, y = node.roll, node.pitch, node.yaw
            fx, fy, fz = node.fx, node.fy, node.fz
            gx, gy  = node.gx, node.gy
            cur_err = err[-1] if err else 0.0
            diag = list(node.diag)
            events = list(node.aruco_events)
            occ = node.occ_grid
            t_now = t[-1] if t else 0.0

        render_path(ax_path, node, fp, gp, cov)
        draw_rover_box(ax_3d, r, p, y)
        render_error(ax_err, err, t)
        render_readout(ax_txt, fx, fy, fz, r, p, y, gx, gy, cur_err, diag)
        render_aruco_timeline(ax_aruco, node, events, t_now)
        render_fis_bars(ax_fis, diag)
        render_map(ax_map, node, occ)
        render_scoreboard(ax_score, err, diag)
        return []

    ani = animation.FuncAnimation(fig, animate, interval=200, blit=False)
    try:
        plt.show()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
