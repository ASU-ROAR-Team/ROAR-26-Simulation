#!/usr/bin/env python3
import json
import math
import re
from datetime import datetime
from collections import deque
from html import escape
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3DArray


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def center_distance(a, b):
    return math.sqrt(
        (a['x'] - b['x']) ** 2 +
        (a['y'] - b['y']) ** 2 +
        (a['z'] - b['z']) ** 2
    )


def aabb_bounds(obj):
    return (
        obj['x'] - obj['length'] / 2.0,
        obj['x'] + obj['length'] / 2.0,
        obj['y'] - obj['width'] / 2.0,
        obj['y'] + obj['width'] / 2.0,
        obj['z'] - obj['height'] / 2.0,
        obj['z'] + obj['height'] / 2.0,
    )



def expanded_aabb_bounds(obj, padding=0.0):
    return (
        obj['x'] - obj['length'] / 2.0 - padding,
        obj['x'] + obj['length'] / 2.0 + padding,
        obj['y'] - obj['width'] / 2.0 - padding,
        obj['y'] + obj['width'] / 2.0 + padding,
        obj['z'] - obj['height'] / 2.0 - padding,
        obj['z'] + obj['height'] / 2.0 + padding,
    )


def point_to_aabb_distance(point, box_obj, padding=0.0):
    bounds = expanded_aabb_bounds(box_obj, padding)
    dx = max(bounds[0] - point['x'], 0.0, point['x'] - bounds[1])
    dy = max(bounds[2] - point['y'], 0.0, point['y'] - bounds[3])
    dz = max(bounds[4] - point['z'], 0.0, point['z'] - bounds[5])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def adaptive_gt_radius(gt, margin):
    half_diagonal = 0.5 * math.sqrt(gt['length'] ** 2 + gt['width'] ** 2 + gt['height'] ** 2)
    return half_diagonal + margin


def iou3d(a, b):
    aa = aabb_bounds(a)
    bb = aabb_bounds(b)
    ix = max(0.0, min(aa[1], bb[1]) - max(aa[0], bb[0]))
    iy = max(0.0, min(aa[3], bb[3]) - max(aa[2], bb[2]))
    iz = max(0.0, min(aa[5], bb[5]) - max(aa[4], bb[4]))
    intersection = ix * iy * iz
    va = max(a['length'] * a['width'] * a['height'], 0.0)
    vb = max(b['length'] * b['width'] * b['height'], 0.0)
    union = va + vb - intersection
    return intersection / union if union > 0.0 else 0.0


def safe_f1(precision, recall):
    return 2.0 * precision * recall / (precision + recall) if precision + recall else 0.0


def extract_marker_id(det):
    if det.results:
        class_id = str(det.results[0].hypothesis.class_id)
        match = re.search(r'(\d+)', class_id)
        if match:
            return int(match.group(1))
    return -1


def percent_error(measured, reference):
    denominator = max(abs(reference), 1e-6)
    return abs(measured - reference) / denominator * 100.0


def fmt(value, digits=3):
    if value is None or not math.isfinite(float(value)):
        return '—'
    return f'{float(value):.{digits}f}'


def pct(value):
    return f'{100.0 * float(value):.1f}%'


def badge(text, kind='neutral'):
    return f'<span class="badge {kind}">{escape(str(text))}</span>'


def svg_bar_chart(title, items, suffix='', maximum=None):
    clean = [(str(label), max(0.0, float(value))) for label, value in items]
    if not clean:
        return '<div class="chart empty">No graph data yet</div>'
    max_value = maximum if maximum is not None else max((value for _, value in clean), default=1.0)
    max_value = max(max_value, 1e-6)
    row_height = 34
    label_width = 210
    chart_width = 620
    value_width = 90
    width = label_width + chart_width + value_width
    height = 48 + row_height * len(clean)
    rows = []
    for index, (label, value) in enumerate(clean):
        y = 40 + index * row_height
        bar_width = chart_width * min(value / max_value, 1.0)
        rows.append(
            f'<text x="8" y="{y + 17}" class="chart-label">{escape(label)}</text>'
            f'<rect x="{label_width}" y="{y}" width="{chart_width}" height="20" rx="3" class="chart-bg"/>'
            f'<rect x="{label_width}" y="{y}" width="{bar_width:.1f}" height="20" rx="3" class="chart-bar"/>'
            f'<text x="{label_width + chart_width + 10}" y="{y + 16}" class="chart-value">{value:.1f}{escape(suffix)}</text>'
        )
    return (
        f'<div class="chart"><div class="chart-title">{escape(title)}</div>'
        f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="{escape(title)}">'
        + ''.join(rows) + '</svg></div>'
    )


def svg_wave_chart(title, series, suffix='%', maximum=100.0):
    """Render a compact multi-series wave chart without external libraries."""
    if not series or not any(values for _, values in series):
        return '<div class="wave-card empty">No history data yet</div>'
    width, height = 920, 280
    left, right, top, bottom = 55, 20, 38, 42
    plot_w, plot_h = width - left - right, height - top - bottom
    colors = ['#e63737', '#2f80ed', '#16a36a', '#f2a900', '#8b5cf6', '#00a6a6']
    grid = []
    for i in range(5):
        y = top + plot_h * i / 4
        value = maximum * (1 - i / 4)
        grid.append(f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" class="wave-grid"/>')
        grid.append(f'<text x="8" y="{y+4:.1f}" class="wave-axis">{value:.0f}{escape(suffix)}</text>')
    paths, legend = [], []
    for idx, (label, values) in enumerate(series):
        if not values:
            continue
        color = colors[idx % len(colors)]
        pts = []
        for i, raw in enumerate(values):
            value = max(0.0, min(float(raw), maximum))
            x = left + plot_w * i / max(len(values)-1, 1)
            y = top + plot_h * (1 - value / max(maximum, 1e-6))
            pts.append((x, y))
        path = ' '.join((('M' if i == 0 else 'L') + f' {x:.1f} {y:.1f}') for i, (x, y) in enumerate(pts))
        area_path = path + f' L {pts[-1][0]:.1f} {height-bottom} L {pts[0][0]:.1f} {height-bottom} Z'
        paths.append(f'<path d="{area_path}" fill="{color}" fill-opacity="0.12" stroke="none"/>')
        paths.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round"/>')
        x, y = pts[-1]
        paths.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="4" fill="{color}"/>')
        legend.append(f'<span><i style="background:{color}"></i>{escape(label)}: {values[-1]:.1f}{escape(suffix)}</span>')
    return (f'<div class="wave-card"><div class="wave-head"><b>{escape(title)}</b><div class="wave-legend">' + ''.join(legend) +
            f'</div></div><svg viewBox="0 0 {width} {height}" role="img">' + ''.join(grid) + ''.join(paths) +
            f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" class="wave-axis-line"/>' +
            '</svg></div>')


class PerceptionBenchmarkNode(Node):
    """Live benchmark with confirmed detections, localization-aware misses and TF health."""

    def __init__(self):
        super().__init__('perception_benchmark_node')
        defaults = {
            'obstacle_input_topic': '/benchmark/perception/world_objects',
            'aruco_input_topic': '/benchmark/perception/world_aruco',
            'tf_status_topic': '/benchmark/perception/tf_status',
            'world_name': 'worldData_126',
            'ground_truth_path': '',
            'aruco_ground_truth_path': '',
            'output_directory': '',
            'target_frame': 'map',
            'base_frame': 'base_link',
            'camera_frame': 'zed2i_depth_optical_frame',
            'match_distance_m': 1.0,
            'gt_box_padding_m': 0.45,
            'duplicate_same_rock_distance_m': 1.25,
            'near_gt_diagnostic_distance_m': 2.0,
            'noise_min_dimension_m': 0.12,
            'noise_min_volume_m3': 0.004,
            'merge_distance_m': 0.85,
            'aruco_merge_distance_m': 0.35,
            'min_confirmations': 3,
            'detection_stale_sec': 8.0,
            'collidable_height_m': 0.30,
            'collidable_volume_m3': 0.05,
            'camera_range_m': 7.0,
            'camera_horizontal_fov_deg': 95.0,
            'camera_vertical_fov_deg': 65.0,
            'min_visible_frames_for_miss': 5,
            'update_period_sec': 1.0,
            'position_error_limit_m': 1.0,
            'aruco_position_error_limit_m': 0.50,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)

        self.world_name = str(self.get_parameter('world_name').value)
        self.gt_path = Path(str(self.get_parameter('ground_truth_path').value)).expanduser()
        self.aruco_gt_path = Path(str(self.get_parameter('aruco_ground_truth_path').value)).expanduser()
        output = str(self.get_parameter('output_directory').value)
        self.output_dir = Path(output).expanduser() if output else (
            Path.home() / 'roar_perception_results' / self.world_name /
            datetime.now().strftime('%Y%m%d_%H%M%S')
        )
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.html_path = self.output_dir / 'dashboard.html'
        self.json_path = self.output_dir / 'results.json'

        self.target_frame = str(self.get_parameter('target_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.match_distance = float(self.get_parameter('match_distance_m').value)
        self.gt_box_padding = float(self.get_parameter('gt_box_padding_m').value)
        self.duplicate_same_rock_distance = float(self.get_parameter('duplicate_same_rock_distance_m').value)
        self.near_gt_diagnostic_distance = float(self.get_parameter('near_gt_diagnostic_distance_m').value)
        self.noise_min_dimension = float(self.get_parameter('noise_min_dimension_m').value)
        self.noise_min_volume = float(self.get_parameter('noise_min_volume_m3').value)
        self.merge_distance = float(self.get_parameter('merge_distance_m').value)
        self.aruco_merge_distance = float(self.get_parameter('aruco_merge_distance_m').value)
        self.min_confirmations = int(self.get_parameter('min_confirmations').value)
        self.stale_sec = float(self.get_parameter('detection_stale_sec').value)
        self.height_threshold = float(self.get_parameter('collidable_height_m').value)
        self.volume_threshold = float(self.get_parameter('collidable_volume_m3').value)
        self.camera_range = float(self.get_parameter('camera_range_m').value)
        self.hfov = math.radians(float(self.get_parameter('camera_horizontal_fov_deg').value))
        self.vfov = math.radians(float(self.get_parameter('camera_vertical_fov_deg').value))
        self.min_visible_frames = int(self.get_parameter('min_visible_frames_for_miss').value)
        self.position_limit = float(self.get_parameter('position_error_limit_m').value)
        self.aruco_position_limit = float(self.get_parameter('aruco_position_error_limit_m').value)

        self.gt = self.load_ground_truth(self.gt_path)
        self.aruco_gt = self.load_aruco_ground_truth(self.aruco_gt_path)
        self.predictions = []
        self.next_prediction_id = 1
        self.aruco_detections = {}
        self.obstacle_frames = 0
        self.aruco_frames = 0
        self.last_obstacle_frame_count = 0
        self.last_obstacle_frame_ros = 0.0
        self.current_visible_gt_count = 0
        self.clear_frame_tn = 0
        self.clear_frame_fp = 0
        self.gt_visibility = {
            item['id']: {'min_robot_distance': float('inf'), 'in_range_frames': 0,
                         'visible_frames': 0, 'last_visible': '', 'ever_in_fov': False}
            for item in self.gt
        }
        self.aruco_visibility = {
            marker_id: {'min_robot_distance': float('inf'), 'in_range_frames': 0,
                        'visible_frames': 0, 'last_visible': '', 'ever_in_fov': False}
            for marker_id in self.aruco_gt
        }
        self.tf_status = {
            'state': 'waiting', 'source_frame': '', 'target_frame': self.target_frame,
            'transformed_obstacles': 0, 'failed_obstacles': 0,
            'transformed_aruco': 0, 'failed_aruco': 0, 'last_error': '',
        }
        self.tf_live = {
            'base_chain_ok': False, 'camera_chain_ok': False,
            'base_error': 'Waiting for TF', 'camera_error': 'Waiting for TF',
            'robot_position': None, 'last_check': '',
        }
        self.metric_history = {
            'precision': deque(maxlen=120), 'recall': deque(maxlen=120),
            'f1': deque(maxlen=120), 'camera_error': deque(maxlen=120),
            'tf_health': deque(maxlen=120), 'position_error': deque(maxlen=120),
            'dimension_error': deque(maxlen=120), 'detections': deque(maxlen=120),
            'system_score': deque(maxlen=120), 'position_score': deque(maxlen=120),
            'dimension_score': deque(maxlen=120), 'collidability_score': deque(maxlen=120),
        }

        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        obstacle_topic = str(self.get_parameter('obstacle_input_topic').value)
        aruco_topic = str(self.get_parameter('aruco_input_topic').value)
        tf_status_topic = str(self.get_parameter('tf_status_topic').value)
        self.create_subscription(Detection3DArray, obstacle_topic, self.obstacle_callback, qos_profile_sensor_data)
        self.create_subscription(Detection3DArray, aruco_topic, self.aruco_callback, qos_profile_sensor_data)
        self.create_subscription(String, tf_status_topic, self.tf_status_callback, 10)
        self.metrics_pub = self.create_publisher(String, '/benchmark/perception/live_metrics', 10)

        period = max(float(self.get_parameter('update_period_sec').value), 0.2)
        self.timer = self.create_timer(period, self.update_outputs)

        self.get_logger().info(f'Loaded {len(self.gt)} obstacle GT objects from {self.gt_path}')
        self.get_logger().info(f'Loaded {len(self.aruco_gt)} ArUco GT markers from {self.aruco_gt_path}')
        self.get_logger().info(
            f'Localization-aware evaluation: {self.target_frame} -> {self.base_frame}, camera={self.camera_frame}')
        self.get_logger().info(f'Live dashboard: {self.html_path}')

    @staticmethod
    def load_ground_truth(path):
        if not path.exists():
            raise FileNotFoundError(f'Ground-truth file not found: {path}')
        loaded = np.load(path, allow_pickle=True)
        records = []
        required = {'id', 'name', 'rock_id', 'frame_id', 'x', 'y', 'z',
                    'length', 'width', 'height', 'is_collidable'}
        for item in loaded:
            obj = item.item() if hasattr(item, 'item') else item
            if not isinstance(obj, dict):
                continue
            missing = required - obj.keys()
            if missing:
                raise ValueError(f'{obj.get("name", "Unknown")} missing fields: {sorted(missing)}')
            records.append({
                'id': int(obj['id']), 'name': str(obj['name']),
                'rock_id': int(obj['rock_id']), 'frame_id': str(obj['frame_id']),
                'x': float(obj['x']), 'y': float(obj['y']), 'z': float(obj['z']),
                'length': float(obj['length']), 'width': float(obj['width']),
                'height': float(obj['height']), 'is_collidable': bool(obj['is_collidable']),
            })
        return records

    @staticmethod
    def load_aruco_ground_truth(path):
        if not path.exists():
            return {}

        try:
            data = yaml.safe_load(
                path.read_text(encoding='utf-8')
            ) or {}

            markers = {}

            for marker in data.get('markers', []):
                marker_id = int(marker['id'])
                position = marker.get('position', {})

                # New GT format:
                #
                # position:
                #   x: ...
                #   y: ...
                #   z: ...
                if isinstance(position, dict):
                    x = float(position.get('x', 0.0))
                    y = float(position.get('y', 0.0))
                    z = float(position.get('z', 0.0))

                # Legacy format:
                #
                # position: [x, y, z]
                elif isinstance(position, (list, tuple)) and len(position) >= 3:
                    x = float(position[0])
                    y = float(position[1])
                    z = float(position[2])

                else:
                    continue

                orientation = marker.get('orientation', {})

                markers[marker_id] = {
                    'id': marker_id,

                    'x': x,
                    'y': y,
                    'z': z,

                    'orientation': {
                        'x': float(orientation.get('x', 0.0))
                        if isinstance(orientation, dict) else 0.0,

                        'y': float(orientation.get('y', 0.0))
                        if isinstance(orientation, dict) else 0.0,

                        'z': float(orientation.get('z', 0.0))
                        if isinstance(orientation, dict) else 0.0,

                        'w': float(orientation.get('w', 1.0))
                        if isinstance(orientation, dict) else 1.0,
                    },

                    'size_m': float(
                        marker.get('size_m', 0.20)
                    ),

                    'frame_id': str(
                        marker.get('frame_id', 'map')
                    ),
                }

            return markers

        except Exception as exc:
            print(
                f'[ArUco GT] Failed to load {path}: {exc}'
            )
            return {}

    def infer_collidable(self, class_id, height, volume):
        label = class_id.lower()
        if any(token in label for token in ('passable', 'non_collidable', 'non-collidable', 'traversable')):
            return False
        if any(token in label for token in ('collidable', 'obstacle', 'rock')):
            return True
        return height >= self.height_threshold or volume >= self.volume_threshold

    def obstacle_callback(self, msg):
        self.obstacle_frames += 1
        now = self.get_clock().now().nanoseconds / 1e9
        self.last_obstacle_frame_count = len(msg.detections)
        self.last_obstacle_frame_ros = now
        for det in msg.detections:
            score = float(det.results[0].hypothesis.score) if det.results else 0.0
            class_id = str(det.results[0].hypothesis.class_id) if det.results else 'object'
            length = float(det.bbox.size.x)
            width = float(det.bbox.size.y)
            height = float(det.bbox.size.z)
            volume = max(length * width * height, 0.0)
            item = {
                'id': self.next_prediction_id, 'frame_id': msg.header.frame_id,
                'x': float(det.bbox.center.position.x),
                'y': float(det.bbox.center.position.y),
                'z': float(det.bbox.center.position.z),
                'length': length, 'width': width, 'height': height,
                'volume': volume, 'confidence': score, 'class_id': class_id,
                'observations': 1, 'first_seen_ros': now, 'last_seen_ros': now,
                'last_seen': datetime.now().isoformat(timespec='seconds'),
                'is_collidable': self.infer_collidable(class_id, height, volume),
            }
            if self.merge_prediction(item):
                self.next_prediction_id += 1

    def merge_prediction(self, new):
        nearest = None
        nearest_distance = float('inf')
        for old in self.predictions:
            distance = center_distance(old, new)
            adaptive_limit = self.merge_distance + 0.15 * max(
                old['length'], old['width'], new['length'], new['width'])
            if distance <= adaptive_limit and distance < nearest_distance:
                nearest = old
                nearest_distance = distance
        if nearest is None:
            self.predictions.append(new)
            return True
        count = nearest['observations']
        alpha = 1.0 / min(count + 1, 12)
        for key in ('x', 'y', 'z', 'length', 'width', 'height', 'confidence'):
            nearest[key] = nearest[key] * (1.0 - alpha) + new[key] * alpha
        nearest['observations'] = count + 1
        nearest['last_seen_ros'] = new['last_seen_ros']
        nearest['last_seen'] = new['last_seen']
        nearest['class_id'] = new['class_id'] or nearest['class_id']
        nearest['volume'] = nearest['length'] * nearest['width'] * nearest['height']
        nearest['is_collidable'] = self.infer_collidable(
            nearest['class_id'], nearest['height'], nearest['volume'])
        return False

    def aruco_callback(self, msg):
        self.aruco_frames += 1
        now = self.get_clock().now().nanoseconds / 1e9
        for det in msg.detections:
            marker_id = extract_marker_id(det)
            if marker_id < 0:
                continue
            score = float(det.results[0].hypothesis.score) if det.results else 0.0
            current = {
                'id': marker_id, 'frame_id': msg.header.frame_id,
                'x': float(det.bbox.center.position.x),
                'y': float(det.bbox.center.position.y),
                'z': float(det.bbox.center.position.z),
                'confidence': score, 'observations': 1,
                'last_seen_ros': now,
                'last_seen': datetime.now().isoformat(timespec='seconds'),
            }
            old = self.aruco_detections.get(marker_id)
            if old is None:
                self.aruco_detections[marker_id] = current
                continue
            count = old['observations']
            if center_distance(old, current) <= self.aruco_merge_distance:
                alpha = 1.0 / min(count + 1, 12)
                for key in ('x', 'y', 'z', 'confidence'):
                    old[key] = old[key] * (1.0 - alpha) + current[key] * alpha
            else:
                for key in ('x', 'y', 'z', 'confidence'):
                    old[key] = current[key]
            old['observations'] = count + 1
            old['last_seen_ros'] = now
            old['last_seen'] = current['last_seen']

    def tf_status_callback(self, msg):
        try:
            received = json.loads(msg.data)
            if isinstance(received, dict):
                self.tf_status.update(received)
        except json.JSONDecodeError:
            self.tf_status['last_error'] = 'Invalid TF status JSON'

    def confirmed_predictions(self):
        return [item for item in self.predictions if item['observations'] >= self.min_confirmations]

    def check_tf_and_visibility(self):
        self.tf_live['last_check'] = datetime.now().isoformat(timespec='seconds')
        robot_position = None
        try:
            base_tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.base_frame, Time(), timeout=Duration(seconds=0.15))
            translation = base_tf.transform.translation
            robot_position = {'x': translation.x, 'y': translation.y, 'z': translation.z}
            self.tf_live.update({'base_chain_ok': True, 'base_error': '', 'robot_position': robot_position})
        except TransformException as exc:
            self.tf_live.update({'base_chain_ok': False, 'base_error': str(exc), 'robot_position': None})

        camera_transform = None
        try:
            camera_transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.target_frame, Time(), timeout=Duration(seconds=0.15))
            self.tf_live.update({'camera_chain_ok': True, 'camera_error': ''})
        except TransformException as exc:
            self.tf_live.update({'camera_chain_ok': False, 'camera_error': str(exc)})

        if robot_position is None or camera_transform is None:
            return
        self.current_visible_gt_count = self.update_visibility_for_objects(self.gt, self.gt_visibility, robot_position, camera_transform)
        marker_objects = [dict(item) for item in self.aruco_gt.values()]
        self.update_visibility_for_objects(marker_objects, self.aruco_visibility, robot_position, camera_transform)

    def update_visibility_for_objects(self, objects, tracking, robot_position, camera_transform):
        now_text = datetime.now().isoformat(timespec='seconds')
        visible_now = 0
        for obj in objects:
            key = obj['id']
            state = tracking[key]
            distance = center_distance(obj, robot_position)
            state['min_robot_distance'] = min(state['min_robot_distance'], distance)
            if distance <= self.camera_range:
                state['in_range_frames'] += 1
            point = PointStamped()
            point.header.frame_id = self.target_frame
            point.point.x, point.point.y, point.point.z = obj['x'], obj['y'], obj['z']
            try:
                camera_point = do_transform_point(point.point, camera_transform)
            except Exception:
                continue
            forward = camera_point.z
            horizontal = abs(math.atan2(camera_point.x, max(forward, 1e-9)))
            vertical = abs(math.atan2(camera_point.y, max(forward, 1e-9)))
            in_fov = (
                forward > 0.0 and distance <= self.camera_range and
                horizontal <= self.hfov / 2.0 and vertical <= self.vfov / 2.0
            )
            if in_fov:
                visible_now += 1
                state['visible_frames'] += 1
                state['ever_in_fov'] = True
                state['last_visible'] = now_text
        return visible_now

    def evaluate_obstacles(self):
        predictions = self.confirmed_predictions()

        # Assign every confirmed detection to the most plausible GT volume.
        # Matching is viewpoint-independent because all detections are already in map frame.
        assignments = {}
        diagnostics = {}
        groups = {}
        for pred_index, pred in enumerate(predictions):
            candidates = []
            nearest = None
            for gt_index, gt in enumerate(self.gt):
                center_d = center_distance(gt, pred)
                box_d = point_to_aabb_distance(pred, gt, self.gt_box_padding)
                overlap = iou3d(gt, pred)
                allowed_center = adaptive_gt_radius(gt, self.match_distance)
                accepted = box_d <= self.match_distance or center_d <= allowed_center or overlap > 0.001
                quality = box_d + 0.20 * center_d - 0.60 * overlap
                entry = {
                    'gt_index': gt_index, 'gt': gt, 'center_distance': center_d,
                    'box_distance': box_d, 'iou': overlap, 'allowed_center': allowed_center,
                    'accepted': accepted, 'quality': quality,
                }
                if nearest is None or (box_d, center_d) < (nearest['box_distance'], nearest['center_distance']):
                    nearest = entry
                if accepted:
                    candidates.append(entry)
            diagnostics[pred_index] = nearest
            if candidates:
                best = min(candidates, key=lambda item: (item['quality'], item['center_distance']))
                assignments[pred_index] = best
                groups.setdefault(best['gt_index'], []).append((pred_index, best))

        # One primary detection track per GT rock. Extra observations from another side/view
        # are attached to the same rock and are not counted as false positives.
        matches = []
        duplicate_predictions = []
        used_pred = set()
        used_gt = set()
        gt_tracks = []
        for gt_index, members in groups.items():
            gt = self.gt[gt_index]
            members.sort(key=lambda pair: (
                pair[1]['box_distance'], pair[1]['center_distance'],
                -predictions[pair[0]]['observations'], -pair[1]['iou']))
            primary_index, primary_info = members[0]
            pred = predictions[primary_index]
            used_pred.add(primary_index)
            used_gt.add(gt_index)

            dim_errors = {
                'length': percent_error(pred['length'], gt['length']),
                'width': percent_error(pred['width'], gt['width']),
                'height': percent_error(pred['height'], gt['height']),
            }
            mean_dim_error = sum(dim_errors.values()) / 3.0
            position_error_pct = primary_info['box_distance'] / max(self.match_distance, 1e-6) * 100.0
            collidable_correct = gt['is_collidable'] == pred['is_collidable']
            position_success = clamp(1.0 - primary_info['box_distance'] / max(self.position_limit, 1e-6))
            dimension_success = clamp(1.0 - mean_dim_error / 100.0)
            detection_success = 100.0 * clamp(
                0.40 * position_success +
                0.25 * dimension_success +
                0.25 * primary_info['iou'] +
                0.10 * float(collidable_correct)
            )
            overall_error_pct = 100.0 - detection_success
            match = {
                'gt': gt, 'pred': pred, 'distance': primary_info['center_distance'],
                'box_distance': primary_info['box_distance'], 'iou': primary_info['iou'],
                'dimension_errors': dim_errors, 'mean_dimension_error_pct': mean_dim_error,
                'position_error_pct': position_error_pct, 'overall_error_pct': overall_error_pct,
                'detection_success_pct': detection_success,
                'collidable_correct': collidable_correct, 'view_count': len(members),
                'total_observations': sum(predictions[i]['observations'] for i, _ in members),
                'match_reason': 'GT VOLUME / ADAPTIVE POSITION MATCH',
            }
            matches.append(match)

            track_members = []
            for member_index, info in members:
                member = predictions[member_index]
                track_members.append(member['id'])
                if member_index == primary_index:
                    continue
                used_pred.add(member_index)
                duplicate_predictions.append({
                    'pred': member, 'gt': gt, 'center_distance': info['center_distance'],
                    'box_distance': info['box_distance'], 'iou': info['iou'],
                    'reason': 'SAME ROCK – DIFFERENT VIEW / DUPLICATE TRACK',
                    'primary_detection_id': pred['id'],
                })
            gt_tracks.append({
                'gt': gt, 'primary_detection_id': pred['id'], 'member_detection_ids': track_members,
                'view_count': len(members), 'total_observations': match['total_observations'],
                'best_box_error_m': min(info['box_distance'] for _, info in members),
                'best_center_error_m': min(info['center_distance'] for _, info in members),
            })

        false_predictions = []
        for pred_index, pred in enumerate(predictions):
            if pred_index in used_pred:
                continue
            nearest = diagnostics.get(pred_index)
            min_dim = min(pred['length'], pred['width'], pred['height'])
            likely_noise = min_dim < self.noise_min_dimension or pred['volume'] < self.noise_min_volume
            if likely_noise:
                reason = 'LIKELY GROUND / THIN CLUSTER NOISE'
            elif nearest and (nearest['box_distance'] <= self.near_gt_diagnostic_distance or
                              nearest['center_distance'] <= nearest['allowed_center'] + self.near_gt_diagnostic_distance):
                reason = 'NEAR GT BUT OUTSIDE ACCEPTED VOLUME / THRESHOLD'
            else:
                reason = 'TRUE FALSE POSITIVE / FAR FROM ALL GT'
            false_predictions.append({
                **pred,
                'nearest_gt': nearest['gt'] if nearest else None,
                'nearest_center_distance': nearest['center_distance'] if nearest else None,
                'nearest_box_distance': nearest['box_distance'] if nearest else None,
                'diagnostic_reason': reason,
            })

        ground_truth_rows = []
        evaluable_gt = missed_gt = not_visited_gt = out_of_fov_gt = 0
        for index, gt in enumerate(self.gt):
            state = self.gt_visibility[gt['id']]
            matched = index in used_gt
            if matched:
                status, kind = 'DETECTED', 'safe'
                evaluable_gt += 1
            elif state['visible_frames'] >= self.min_visible_frames:
                status, kind = 'MISSED', 'danger'
                evaluable_gt += 1
                missed_gt += 1
            elif state['in_range_frames'] > 0:
                status, kind = 'IN RANGE / OUT OF FOV', 'warning'
                out_of_fov_gt += 1
            else:
                status, kind = 'NOT VISITED', 'neutral'
                not_visited_gt += 1
            ground_truth_rows.append({'gt': gt, 'visibility': state, 'matched': matched,
                                      'status': status, 'kind': kind})

        tp = len(matches)
        fp = len(false_predictions)
        fn = missed_gt
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        f1 = safe_f1(precision, recall)
        false_discovery = fp / (tp + fp) if tp + fp else 0.0
        miss_rate = fn / (tp + fn) if tp + fn else 0.0
        mean_iou = sum(m['iou'] for m in matches) / tp if tp else 0.0
        mean_position = sum(m['box_distance'] for m in matches) / tp if tp else 0.0
        max_position = max((m['box_distance'] for m in matches), default=0.0)
        mean_dimension = sum(m['mean_dimension_error_pct'] for m in matches) / tp if tp else 0.0
        camera_error_rate = (fp + fn) / (tp + fp + fn) if tp + fp + fn else 0.0
        collidable_correct = sum(1 for m in matches if m['collidable_correct'])
        collidability_accuracy = collidable_correct / tp if tp else 0.0
        position_score = clamp(1.0 - mean_position / max(self.position_limit, 1e-6)) if tp else 0.0
        dimension_score = clamp(1.0 - mean_dimension / 100.0) if tp else 0.0
        mean_detection_success = sum(m['detection_success_pct'] for m in matches) / tp if tp else 0.0
        return {
            'predictions': predictions, 'matches': matches,
            'duplicate_predictions': duplicate_predictions, 'gt_tracks': gt_tracks,
            'false_predictions': false_predictions, 'ground_truth_rows': ground_truth_rows,
            'tp': tp, 'fp': fp, 'fn': fn, 'precision': precision, 'recall': recall,
            'f1': f1, 'false_discovery_rate': false_discovery, 'miss_rate': miss_rate,
            'camera_error_rate': camera_error_rate, 'mean_iou': mean_iou,
            'mean_position_error_m': mean_position, 'max_position_error_m': max_position,
            'mean_dimension_error_pct': mean_dimension,
            'collidability_accuracy': collidability_accuracy,
            'evaluable_gt': evaluable_gt, 'missed_gt': missed_gt,
            'not_visited_gt': not_visited_gt, 'out_of_fov_gt': out_of_fov_gt,
            'pending_predictions': len(self.predictions) - len(predictions),
            'tn': self.clear_frame_tn, 'clear_frame_fp': self.clear_frame_fp,
            'position_score': position_score, 'dimension_score': dimension_score,
            'mean_detection_success_pct': mean_detection_success,
            'duplicate_count': len(duplicate_predictions), 'unique_gt_tracks': len(gt_tracks),
        }

    def evaluate_aruco(self):
        confirmed = {marker_id: item for marker_id, item in self.aruco_detections.items()
                     if item['observations'] >= self.min_confirmations}
        rows = []
        tp = fp = fn = 0
        position_errors = []
        for marker_id, detected in sorted(confirmed.items()):
            gt = self.aruco_gt.get(marker_id)
            if gt:
                distance = center_distance(detected, gt)
                tp += 1
                position_errors.append(distance)
                status = 'CORRECT MARKER' if distance <= self.aruco_position_limit else 'POSITION INACCURATE'
                kind = 'safe' if distance <= self.aruco_position_limit else 'warning'
            else:
                distance = None
                fp += 1
                status, kind = 'UNMAPPED / WRONG ID', 'danger'
            rows.append({'id': marker_id, 'detected': detected, 'gt': gt,
                         'distance': distance, 'status': status, 'kind': kind})
        for marker_id, gt in sorted(self.aruco_gt.items()):
            if marker_id in confirmed:
                continue
            state = self.aruco_visibility[marker_id]
            if state['visible_frames'] >= self.min_visible_frames:
                status, kind = 'MISSED MARKER', 'danger'
                fn += 1
            elif state['in_range_frames'] > 0:
                status, kind = 'IN RANGE / OUT OF FOV', 'warning'
            else:
                status, kind = 'NOT VISITED', 'neutral'
            rows.append({'id': marker_id, 'detected': None, 'gt': gt, 'distance': None,
                         'status': status, 'kind': kind, 'visibility': state})
        precision = tp / (tp + fp) if tp + fp else 0.0
        recall = tp / (tp + fn) if tp + fn else 0.0
        return {
            'rows': rows, 'tp': tp, 'fp': fp, 'fn': fn,
            'precision': precision, 'recall': recall, 'f1': safe_f1(precision, recall),
            'mean_position_error_m': sum(position_errors) / len(position_errors) if position_errors else 0.0,
            'max_position_error_m': max(position_errors, default=0.0),
            'confirmed_count': len(confirmed),
        }

    def evaluate_tf(self):
        obstacle_total = int(self.tf_status.get('transformed_obstacles', 0)) + int(self.tf_status.get('failed_obstacles', 0))
        aruco_total = int(self.tf_status.get('transformed_aruco', 0)) + int(self.tf_status.get('failed_aruco', 0))
        obstacle_success = int(self.tf_status.get('transformed_obstacles', 0)) / obstacle_total if obstacle_total else 0.0
        aruco_success = int(self.tf_status.get('transformed_aruco', 0)) / aruco_total if aruco_total else 0.0
        chain_score = (float(self.tf_live['base_chain_ok']) + float(self.tf_live['camera_chain_ok'])) / 2.0
        streams = []
        if obstacle_total:
            streams.append(obstacle_success)
        if aruco_total:
            streams.append(aruco_success)
        stream_score = sum(streams) / len(streams) if streams else 0.0
        overall = 0.6 * stream_score + 0.4 * chain_score
        return {
            'obstacle_success_rate': obstacle_success,
            'aruco_success_rate': aruco_success,
            'overall_success_rate': overall,
            'base_chain_ok': self.tf_live['base_chain_ok'],
            'camera_chain_ok': self.tf_live['camera_chain_ok'],
            'base_error': self.tf_live['base_error'],
            'camera_error': self.tf_live['camera_error'],
            'robot_position': self.tf_live['robot_position'],
            'last_error': str(self.tf_status.get('last_error', '')),
            'transformed_obstacles': int(self.tf_status.get('transformed_obstacles', 0)),
            'failed_obstacles': int(self.tf_status.get('failed_obstacles', 0)),
            'transformed_aruco': int(self.tf_status.get('transformed_aruco', 0)),
            'failed_aruco': int(self.tf_status.get('failed_aruco', 0)),
        }

    def evaluate(self):
        obstacle = self.evaluate_obstacles()
        aruco = self.evaluate_aruco()
        tf_result = self.evaluate_tf()
        return {
            'timestamp': datetime.now().isoformat(timespec='seconds'),
            'world_name': self.world_name,
            'obstacle_frames': self.obstacle_frames,
            'aruco_frames': self.aruco_frames,
            'obstacle': obstacle, 'aruco': aruco, 'tf': tf_result,
        }

    def update_outputs(self):
        self.check_tf_and_visibility()
        result = self.evaluate()
        obstacle = result['obstacle']
        tf_result = result['tf']
        now_ros = self.get_clock().now().nanoseconds / 1e9
        if self.tf_live['camera_chain_ok'] and now_ros - self.last_obstacle_frame_ros <= 2.5 and self.current_visible_gt_count == 0:
            if self.last_obstacle_frame_count == 0:
                self.clear_frame_tn += 1
            else:
                self.clear_frame_fp += 1
            obstacle['tn'] = self.clear_frame_tn
            obstacle['clear_frame_fp'] = self.clear_frame_fp
        system_score = 100.0 * clamp(
            0.35 * obstacle['f1'] +
            0.20 * obstacle['position_score'] +
            0.15 * obstacle['dimension_score'] +
            0.10 * obstacle['collidability_accuracy'] +
            0.10 * obstacle['mean_iou'] +
            0.10 * tf_result['overall_success_rate']
        )
        result['system_score'] = system_score
        self.metric_history['precision'].append(obstacle['precision'] * 100.0)
        self.metric_history['recall'].append(obstacle['recall'] * 100.0)
        self.metric_history['f1'].append(obstacle['f1'] * 100.0)
        self.metric_history['camera_error'].append(obstacle['camera_error_rate'] * 100.0)
        self.metric_history['tf_health'].append(tf_result['overall_success_rate'] * 100.0)
        self.metric_history['position_error'].append(obstacle['mean_position_error_m'])
        self.metric_history['dimension_error'].append(obstacle['mean_dimension_error_pct'])
        self.metric_history['detections'].append(float(len(obstacle['predictions'])))
        self.metric_history['system_score'].append(system_score)
        self.metric_history['position_score'].append(obstacle['position_score'] * 100.0)
        self.metric_history['dimension_score'].append(obstacle['dimension_score'] * 100.0)
        self.metric_history['collidability_score'].append(obstacle['collidability_accuracy'] * 100.0)
        serializable = self.serialize_result(result)
        self.json_path.write_text(json.dumps(serializable, indent=2), encoding='utf-8')
        self.html_path.write_text(self.render_html(result), encoding='utf-8')
        if rclpy.ok():
            message = String()
            message.data = json.dumps({
                'obstacle_f1': result['obstacle']['f1'],
                'camera_error_rate': result['obstacle']['camera_error_rate'],
                'aruco_f1': result['aruco']['f1'],
                'tf_success_rate': result['tf']['overall_success_rate'],
            })
            self.metrics_pub.publish(message)

    def serialize_result(self, result):
        obstacle = result['obstacle']
        aruco = result['aruco']
        return {
            'timestamp': result['timestamp'], 'world_name': result['world_name'],
            'obstacle_frames': result['obstacle_frames'], 'aruco_frames': result['aruco_frames'],
            'obstacle_metrics': {k: v for k, v in obstacle.items()
                                 if k not in ('predictions', 'matches', 'duplicate_predictions', 'gt_tracks', 'false_predictions', 'ground_truth_rows')},
            'detected_obstacles': obstacle['predictions'],
            'matches': [{
                'detection_id': m['pred']['id'], 'gt_id': m['gt']['id'],
                'gt_name': m['gt']['name'], 'position_error_m': m['distance'],
                'position_error_pct': m['position_error_pct'], 'iou3d': m['iou'],
                'dimension_error_pct': m['mean_dimension_error_pct'],
                'collidability_correct': m['collidable_correct'],
                'overall_error_pct': m['overall_error_pct'],
                'detection_success_pct': m['detection_success_pct'],
            } for m in obstacle['matches']],
            'duplicate_same_rock_detections': obstacle['duplicate_predictions'],
            'gt_tracks': obstacle['gt_tracks'],
            'false_obstacles': obstacle['false_predictions'],
            'ground_truth_evaluation': [{
                'gt_id': row['gt']['id'], 'name': row['gt']['name'],
                'status': row['status'], 'visibility': row['visibility'],
            } for row in obstacle['ground_truth_rows']],
            'aruco_metrics': {k: v for k, v in aruco.items() if k != 'rows'},
            'aruco_rows': [{
                'id': row['id'], 'status': row['status'], 'distance': row['distance'],
            } for row in aruco['rows']],
            'tf': result['tf'],
        }

    def render_html(self, result):
        obstacle, aruco, tf_result = result['obstacle'], result['aruco'], result['tf']
        match_by_pred = {m['pred']['id']: m for m in obstacle['matches']}
        false_ids = {p['id'] for p in obstacle['false_predictions']}
        duplicate_by_pred = {d['pred']['id']: d for d in obstacle.get('duplicate_predictions', [])}

        detection_rows = ''
        for pred in obstacle['predictions']:
            match = match_by_pred.get(pred['id'])
            if match:
                validation = badge(f'CORRECT · {match["gt"]["name"]}', 'safe')
            elif pred['id'] in duplicate_by_pred:
                dup = duplicate_by_pred[pred['id']]
                validation = badge(f'SAME {dup["gt"]["name"]} · ANOTHER VIEW', 'warning')
            elif pred['id'] in false_ids:
                validation = badge('FALSE / UNMATCHED', 'danger')
            else:
                validation = badge('WAITING', 'neutral')
            detection_rows += (
                '<tr>'
                f'<td>OBJ-{pred["id"]:03d}</td><td>{escape(pred["last_seen"])}</td>'
                f'<td>{fmt(pred["x"])}</td><td>{fmt(pred["y"])}</td><td>{fmt(pred["z"])}</td>'
                f'<td>{fmt(pred["length"])}</td><td>{fmt(pred["width"])}</td><td>{fmt(pred["height"])}</td>'
                f'<td>{fmt(pred["volume"])}</td><td>{pct(pred["confidence"])}</td><td>{pred["observations"]}</td>'
                f'<td>{badge("COLLIDABLE" if pred["is_collidable"] else "PASSABLE", "danger" if pred["is_collidable"] else "safe")}</td>'
                f'<td>{validation}</td></tr>'
            )
        if not detection_rows:
            detection_rows = '<tr><td colspan="14">No confirmed object yet. New rows appear only after multi-frame confirmation.</td></tr>'

        comparison_rows = ''
        for match in obstacle['matches']:
            comparison_rows += (
                '<tr>'
                f'<td>OBJ-{match["pred"]["id"]:03d}</td><td>{escape(match["gt"]["name"])}</td>'
                f'<td>{fmt(match["pred"]["x"])}, {fmt(match["pred"]["y"])}, {fmt(match["pred"]["z"])}</td>'
                f'<td>{fmt(match["gt"]["x"])}, {fmt(match["gt"]["y"])}, {fmt(match["gt"]["z"])}</td>'
                f'<td>{fmt(match["box_distance"])} m box / {fmt(match["distance"])} m center</td><td>{match["position_error_pct"]:.1f}%</td>'
                f'<td>{fmt(match["pred"]["length"])}/{fmt(match["pred"]["width"])}/{fmt(match["pred"]["height"])}</td>'
                f'<td>{fmt(match["gt"]["length"])}/{fmt(match["gt"]["width"])}/{fmt(match["gt"]["height"])}</td>'
                f'<td>{match["mean_dimension_error_pct"]:.1f}%</td><td>{pct(match["iou"])}</td>'
                f'<td>{badge("CORRECT" if match["collidable_correct"] else "WRONG", "safe" if match["collidable_correct"] else "danger")}</td>'
                f'<td>{match["overall_error_pct"]:.1f}%</td><td><b>{match["detection_success_pct"]:.1f}%</b></td>'
                f'<td>{badge("CORRECT · {} VIEW TRACK".format(match["view_count"]), "safe")}</td></tr>'
            )
        for duplicate in obstacle.get('duplicate_predictions', []):
            pred, gt = duplicate['pred'], duplicate['gt']
            comparison_rows += (
                '<tr>'
                f'<td>OBJ-{pred["id"]:03d}</td><td>{escape(gt["name"])}</td>'
                f'<td>{fmt(pred["x"])}, {fmt(pred["y"])}, {fmt(pred["z"])}</td>'
                f'<td>{fmt(gt["x"])}, {fmt(gt["y"])}, {fmt(gt["z"])}</td>'
                f'<td>{fmt(duplicate["box_distance"])} m box / {fmt(duplicate["center_distance"])} m center</td><td>—</td>'
                f'<td>{fmt(pred["length"])}/{fmt(pred["width"])}/{fmt(pred["height"])}</td>'
                f'<td>{fmt(gt["length"])}/{fmt(gt["width"])}/{fmt(gt["height"])}</td>'
                f'<td>—</td><td>{pct(duplicate["iou"])}</td><td>—</td><td>Not counted as FP</td><td>Track update</td>'
                f'<td>{badge("SAME ROCK · DIFFERENT VIEW", "warning")}</td></tr>'
            )
        for pred in obstacle['false_predictions']:
            nearest = pred.get('nearest_gt')
            nearest_name = escape(nearest['name']) if nearest else '—'
            distance_text = f'{fmt(pred.get("nearest_box_distance"))} m box / {fmt(pred.get("nearest_center_distance"))} m center' if nearest else '—'
            comparison_rows += (
                '<tr>'
                f'<td>OBJ-{pred["id"]:03d}</td><td>{nearest_name}</td><td>{fmt(pred["x"])}, {fmt(pred["y"])}, {fmt(pred["z"])}</td>'
                f'<td>{(fmt(nearest["x"]) + ", " + fmt(nearest["y"]) + ", " + fmt(nearest["z"])) if nearest else "—"}</td>'
                f'<td>{distance_text}</td><td>100.0%</td>'
                f'<td>{fmt(pred["length"])}/{fmt(pred["width"])}/{fmt(pred["height"])}</td>'
                f'<td>{(fmt(nearest["length"]) + "/" + fmt(nearest["width"]) + "/" + fmt(nearest["height"])) if nearest else "—"}</td>'
                '<td>—</td><td>0.0%</td><td>—</td><td>100.0%</td><td>0.0%</td>'
                f'<td>{badge(pred.get("diagnostic_reason", "FALSE / UNMATCHED"), "danger")}</td></tr>'
            )
        if not comparison_rows:
            comparison_rows = '<tr><td colspan="14">No confirmed GT comparison yet.</td></tr>'

        gt_rows = ''
        for row in obstacle['ground_truth_rows']:
            gt, state = row['gt'], row['visibility']
            min_distance = state['min_robot_distance'] if math.isfinite(state['min_robot_distance']) else None
            gt_rows += (
                '<tr>'
                f'<td>{gt["id"]}</td><td>{escape(gt["name"])}</td>'
                f'<td>{fmt(gt["x"])}, {fmt(gt["y"])}, {fmt(gt["z"])}</td>'
                f'<td>{fmt(min_distance)} m</td><td>{state["in_range_frames"]}</td><td>{state["visible_frames"]}</td>'
                f'<td>{badge(row["status"], row["kind"])}</td></tr>'
            )

        # ============================================================
        # ArUco Benchmark Dashboard
        # ============================================================
        aruco_rows_html = ''

        for row in aruco['rows']:
            detected = row.get('detected')
            gt = row.get('gt')
            visibility = row.get('visibility', {})

            detected_pos = (
                f'{fmt(detected["x"])}, '
                f'{fmt(detected["y"])}, '
                f'{fmt(detected["z"])}'
                if detected else '—'
            )

            gt_pos = (
                f'{fmt(gt["x"])}, '
                f'{fmt(gt["y"])}, '
                f'{fmt(gt["z"])}'
                if gt else '—'
            )

            observations = (
                detected.get('observations', 0)
                if detected else 0
            )

            confidence = (
                pct(detected.get('confidence', 0.0))
                if detected else '—'
            )

            visible_frames = visibility.get(
                'visible_frames', '—'
            )

            in_range_frames = visibility.get(
                'in_range_frames', '—'
            )

            aruco_rows_html += (
                '<tr>'
                f'<td>{row["id"]}</td>'
                f'<td>{detected_pos}</td>'
                f'<td>{gt_pos}</td>'
                f'<td>{fmt(row.get("distance"))} m</td>'
                f'<td>{confidence}</td>'
                f'<td>{observations}</td>'
                f'<td>{in_range_frames}</td>'
                f'<td>{visible_frames}</td>'
                f'<td>{badge(row["status"], row["kind"])}</td>'
                '</tr>'
            )

        if not aruco_rows_html:
            aruco_rows_html = (
                '<tr><td colspan="9">'
                'No ArUco GT or detections available.'
                '</td></tr>'
            )

        aruco_cards = ''.join([
            f'<div class="card"><small>ArUco TP</small>'
            f'<strong>{aruco["tp"]}</strong></div>',

            f'<div class="card"><small>ArUco FP</small>'
            f'<strong>{aruco["fp"]}</strong></div>',

            f'<div class="card"><small>ArUco FN</small>'
            f'<strong>{aruco["fn"]}</strong></div>',

            f'<div class="card"><small>Precision</small>'
            f'<strong>{pct(aruco["precision"])}</strong></div>',

            f'<div class="card"><small>Recall</small>'
            f'<strong>{pct(aruco["recall"])}</strong></div>',

            f'<div class="card"><small>F1</small>'
            f'<strong>{pct(aruco["f1"])}</strong></div>',

            f'<div class="card"><small>Mean Position Error</small>'
            f'<strong>{fmt(aruco["mean_position_error_m"])} m</strong></div>',

            f'<div class="card"><small>Confirmed Markers</small>'
            f'<strong>{aruco["confirmed_count"]}</strong></div>',

            f'<div class="card"><small>ArUco TF Success</small>'
            f'<strong>{pct(tf_result["aruco_success_rate"])}</strong></div>',
        ])

        aruco_chart = svg_bar_chart(
            'ArUco benchmark scores',
            [
                ('Precision', aruco['precision'] * 100),
                ('Recall', aruco['recall'] * 100),
                ('F1', aruco['f1'] * 100),
                ('TF success',
                 tf_result['aruco_success_rate'] * 100),
            ],
            '%',
            100
        )

        robot = tf_result['robot_position']
        tf_rows = (
            f'<tr><td>{self.target_frame} → {self.base_frame}</td><td>{badge("OK" if tf_result["base_chain_ok"] else "FAILED", "safe" if tf_result["base_chain_ok"] else "danger")}</td><td>{escape(tf_result["base_error"] or "Transform available")}</td></tr>'
            f'<tr><td>{self.target_frame} → {self.camera_frame}</td><td>{badge("OK" if tf_result["camera_chain_ok"] else "FAILED", "safe" if tf_result["camera_chain_ok"] else "danger")}</td><td>{escape(tf_result["camera_error"] or "Transform available")}</td></tr>'
            f'<tr><td>Obstacle TF stream</td><td>{pct(tf_result["obstacle_success_rate"])}</td><td>{tf_result["transformed_obstacles"]} transformed / {tf_result["failed_obstacles"]} failed</td></tr>'
            f'<tr><td>Robot localization</td><td>{badge("AVAILABLE" if robot else "UNAVAILABLE", "safe" if robot else "danger")}</td><td>{fmt(robot["x"] if robot else None)}, {fmt(robot["y"] if robot else None)}, {fmt(robot["z"] if robot else None)}</td></tr>'
            f'<tr><td>Last TF error</td><td colspan="2">{escape(tf_result["last_error"] or "None")}</td></tr>'
        )

        score = float(result.get('system_score', 0.0))
        cards = [
            ('Overall system score', f'{score:.1f}%'), ('Raw confirmed objects', len(obstacle['predictions'])), ('Unique GT tracks', obstacle.get('unique_gt_tracks', 0)), ('Same-rock duplicates', obstacle.get('duplicate_count', 0)),
            ('TP', obstacle['tp']), ('FP', obstacle['fp']), ('FN', obstacle['fn']),
            ('TN · clear frames', obstacle['tn']),
            ('Precision', pct(obstacle['precision'])), ('Recall', pct(obstacle['recall'])), ('F1', pct(obstacle['f1'])),
            ('Position score', pct(obstacle['position_score'])), ('Dimension score', pct(obstacle['dimension_score'])),
            ('Collidability score', pct(obstacle['collidability_accuracy'])), ('Mean 3D IoU', pct(obstacle['mean_iou'])),
            ('Mean detection success', f'{obstacle["mean_detection_success_pct"]:.1f}%'),
            ('TF health', pct(tf_result['overall_success_rate'])),
        ]
        card_html = ''.join(f'<div class="card"><small>{escape(str(t))}</small><strong>{escape(str(v))}</strong></div>' for t, v in cards)

        wave_score = svg_wave_chart('Overall score wave', [('System score', list(self.metric_history['system_score']))])
        wave_quality = svg_wave_chart('Detection quality wave', [
            ('Precision', list(self.metric_history['precision'])), ('Recall', list(self.metric_history['recall'])),
            ('F1', list(self.metric_history['f1']))])
        wave_components = svg_wave_chart('Component score wave', [
            ('Position', list(self.metric_history['position_score'])), ('Dimensions', list(self.metric_history['dimension_score'])),
            ('Collidability', list(self.metric_history['collidability_score'])), ('TF health', list(self.metric_history['tf_health']))])
        wave_error = svg_wave_chart('Live error history', [
            ('Camera error', list(self.metric_history['camera_error'])),
            ('Dimension error', list(self.metric_history['dimension_error']))])
        max_pos = max(list(self.metric_history['position_error']) or [1.0])
        wave_position = svg_wave_chart('Mean position error over time', [('Position error', list(self.metric_history['position_error']))], ' m', max(1.0, max_pos * 1.2))
        decisions = svg_bar_chart('Current detection decisions', [('Unique correct rocks', obstacle['tp']), ('Same-rock extra views', obstacle.get('duplicate_count', 0)), ('True unmatched / noise', obstacle['fp']), ('Pending', obstacle['pending_predictions'])])
        coverage = svg_bar_chart('Ground-truth coverage', [('Detected', obstacle['tp']), ('Missed after visibility', obstacle['missed_gt']), ('Out of FOV', obstacle['out_of_fov_gt']), ('Not visited', obstacle['not_visited_gt'])])
        tf_chart = svg_bar_chart('TF health', [('Obstacle transforms', tf_result['obstacle_success_rate'] * 100), ('ArUco transforms', tf_result['aruco_success_rate'] * 100), ('Overall', tf_result['overall_success_rate'] * 100)], '%', 100)
        needs = []
        if obstacle['fp'] > obstacle['tp']:
            needs.append('Reduce false positives: increase multi-frame confirmation and tune cluster size / ground-removal filters.')
        if obstacle['position_score'] < 0.70:
            needs.append('Improve position accuracy: verify camera extrinsics, localization alignment and timestamp synchronization.')
        if obstacle['dimension_score'] < 0.70:
            needs.append('Improve bounding-box dimensions: tune clustering and remove partial / thin clusters.')
        if obstacle['collidability_accuracy'] < 0.80 and obstacle['tp']:
            needs.append('Tune collidability thresholds using GT height and volume.')
        if tf_result['overall_success_rate'] < 0.95:
            needs.append('Fix TF timing or missing frame chains before trusting perception scores.')
        if not needs:
            needs.append('System is healthy at the current checkpoint; continue testing more rocks and viewpoints.')
        needs_html = '<ul class="needs">' + ''.join(f'<li>{escape(item)}</li>' for item in needs) + '</ul>'

        return f'''<!DOCTYPE html><html lang="en"><head><meta charset="utf-8"><meta http-equiv="refresh" content="1"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ROAR Perception Benchmark</title>
<style>
:root{{--bg:#f4f6f8;--panel:#fff;--dark:#101216;--red:#e63737;--line:#dfe3e8;--muted:#6f7782}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:#1d232b;font-family:Inter,Arial,sans-serif}}.top{{background:var(--dark);color:#fff;border-bottom:4px solid var(--red);position:sticky;top:0;z-index:20}}.top-inner{{max-width:1700px;margin:auto;padding:18px 28px}}h1{{margin:0;font-size:23px}}h1 span{{color:var(--red)}}.sub{{color:#aeb5be;margin-top:5px;font-size:12px}}
.score-strip{{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:1px;background:#343941;margin-top:14px}}.score-strip div{{background:#171a20;padding:10px 12px}}.score-strip small{{display:block;color:#9ea6b0;font-size:10px;text-transform:uppercase}}.score-strip b{{display:block;margin-top:4px;font-size:17px}}
.tabs{{display:flex;gap:7px;overflow:auto;padding:12px 0 0}}.tab{{border:1px solid #3a4049;color:#d8dde3;background:#1b1f25;padding:9px 14px;border-radius:7px 7px 0 0;cursor:pointer;white-space:nowrap}}.tab.active{{background:var(--red);border-color:var(--red);color:#fff}}
.container{{max-width:1700px;margin:auto;padding:24px 28px}}.page{{display:none}}.page.active{{display:block}}.page-title{{display:flex;justify-content:space-between;align-items:end;gap:20px;margin-bottom:16px}}.page-title h2{{margin:0;font-size:23px}}.page-title p{{margin:5px 0 0;color:var(--muted);font-size:13px}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px;margin-bottom:16px}}.card{{background:var(--panel);border:1px solid var(--line);border-top:4px solid var(--red);padding:15px;border-radius:7px}}.card small{{display:block;color:var(--muted);font-size:10px;text-transform:uppercase;letter-spacing:.6px}}.card strong{{display:block;font-size:22px;margin-top:7px}}
.panel{{background:var(--panel);border:1px solid var(--line);border-radius:8px;margin-bottom:18px;padding:16px;box-shadow:0 2px 8px rgba(0,0,0,.03)}}.table-wrap{{overflow:auto;border:1px solid var(--line);border-radius:7px}}table{{width:100%;border-collapse:collapse;min-width:1150px}}th{{background:#15181d;color:#fff;padding:10px 8px;font-size:10px;text-transform:uppercase}}td{{padding:9px 8px;border-bottom:1px solid #e5e8ec;text-align:center;font-size:12px}}tr:nth-child(even) td{{background:#fafbfc}}.badge{{display:inline-block;padding:4px 7px;border-radius:4px;font-size:10px;font-weight:800}}.safe{{background:#dcf6e8;color:#117344}}.danger{{background:#ffe0e0;color:#ae1620}}.warning{{background:#fff1cd;color:#895c00}}.neutral{{background:#e9edf1;color:#49515b}}
.wave-grid{{stroke:#e3e7eb;stroke-width:1}}.wave-axis{{font-size:10px;fill:#747d87}}.wave-axis-line{{stroke:#aab2bb}}.graph-grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(520px,1fr));gap:16px}}.wave-card{{background:#fff;border:1px solid var(--line);border-radius:8px;padding:14px;margin-bottom:16px;overflow:auto}}.wave-card svg{{min-width:760px;width:100%;height:auto}}.wave-head{{display:flex;justify-content:space-between;gap:15px;flex-wrap:wrap}}.wave-legend{{display:flex;gap:12px;flex-wrap:wrap;font-size:11px;color:#4e5660}}.wave-legend i{{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px}}
.chart{{margin-top:14px;border:1px solid var(--line);padding:14px;background:#fff;overflow:auto;border-radius:8px}}.chart-title{{font-size:13px;font-weight:800;margin-bottom:6px}}.chart svg{{min-width:800px;max-width:100%;height:auto}}.chart-label{{font-size:12px;fill:#333}}.chart-value{{font-size:12px;font-weight:700;fill:#222}}.chart-bg{{fill:#eceff2}}.chart-bar{{fill:#e63737}}.needs{{margin:0;padding-left:22px;line-height:1.8}}.placeholder{{padding:70px 20px;text-align:center;border:2px dashed #cfd5dc;border-radius:9px;background:#fafbfc;color:#68717c}}.footer{{text-align:center;color:#777;font-size:11px;padding:18px}}
</style></head><body>
<div class="top"><div class="top-inner"><h1>ASU ROAR — <span>Perception Benchmark</span></h1><div class="sub">Viewpoint-independent workflow: camera detection → TF to map → adaptive GT-volume matching → one track per Rock ID</div>
<div class="score-strip"><div><small>Overall Score</small><b>{score:.1f}%</b></div><div><small>TP</small><b>{obstacle['tp']}</b></div><div><small>FP</small><b>{obstacle['fp']}</b></div><div><small>FN</small><b>{obstacle['fn']}</b></div><div><small>TN · Clear Frames</small><b>{obstacle['tn']}</b></div><div><small>F1</small><b>{pct(obstacle['f1'])}</b></div><div><small>TF Health</small><b>{pct(tf_result['overall_success_rate'])}</b></div></div>
<div class="tabs"><button class="tab active" onclick="showPage('score',this)">1. Scoring</button><button class="tab" onclick="showPage('objects',this)">2. All Objects</button><button class="tab" onclick="showPage('gt',this)">3. GT Comparison</button><button class="tab" onclick="showPage('aruco',this)">4. ArUco</button><button class="tab" onclick="showPage('system',this)">5. Full System</button></div></div></div>
<div class="container">
<section id="score" class="page active"><div class="page-title"><div><h2>Live Scoring Dashboard</h2><p>Overall score, confusion counts, component scores and live area-wave graphs.</p></div><div>{escape(result['timestamp'])}</div></div><div class="cards">{card_html}</div><div class="graph-grid">{wave_score}{wave_quality}{wave_components}{wave_error}{wave_position}</div><div class="panel"><h3>What the system needs now</h3>{needs_html}</div><div class="panel"><small><b>TN definition:</b> clear camera frames where TF is valid, no GT rock is inside the current FOV, and the perception publishes no object. Object detection does not have a natural object-level TN without defining negative regions.</small></div></section>
<section id="objects" class="page"><div class="page-title"><div><h2>All Confirmed Objects</h2><p>Raw confirmed clusters are shown here; another viewpoint of the same GT rock is attached to one Rock-ID track and is not counted as FP.</p></div></div><div class="panel"><div class="table-wrap"><table><thead><tr><th>ID</th><th>Last Seen</th><th>Map X</th><th>Map Y</th><th>Map Z</th><th>Length</th><th>Width</th><th>Height</th><th>Volume</th><th>Confidence</th><th>Observations</th><th>Decision</th><th>GT Validation</th></tr></thead><tbody>{detection_rows}</tbody></table></div>{decisions}</div></section>
<section id="gt" class="page"><div class="page-title"><div><h2>Detection vs Ground Truth</h2><p>Map-frame values are matched against expanded GT volumes. Side/front/rear views can resolve to the same Rock ID.</p></div></div><div class="panel"><div class="table-wrap"><table><thead><tr><th>Detection</th><th>Matched GT</th><th>Detected Position</th><th>GT Position</th><th>Position Error</th><th>Position Error %</th><th>Detected L/W/H</th><th>GT L/W/H</th><th>Dimension Error</th><th>3D IoU</th><th>Collidability</th><th>Overall Error</th><th>Detection Success</th><th>Result</th></tr></thead><tbody>{comparison_rows}</tbody></table></div>{coverage}</div><div class="panel"><h3>Localization-aware GT coverage</h3><div class="table-wrap"><table><thead><tr><th>GT ID</th><th>Rock</th><th>GT Position</th><th>Closest Robot Distance</th><th>Frames In Range</th><th>Frames In Camera FOV</th><th>Status</th></tr></thead><tbody>{gt_rows}</tbody></table></div></div></section>
<section id="aruco" class="page">
<div class="page-title">
<div>
<h2>ArUco Benchmark</h2>
<p>
ID matching + map-frame position accuracy +
localization-aware missed-marker evaluation.
</p>
</div>
<div>Frames: {result['aruco_frames']}</div>
</div>

<div class="cards">
{aruco_cards}
</div>

<div class="panel">
{aruco_chart}
</div>

<div class="panel">
<div class="table-wrap">
<table>
<thead>
<tr>
<th>Marker ID</th>
<th>Detected Position</th>
<th>GT Position</th>
<th>Position Error</th>
<th>Confidence</th>
<th>Observations</th>
<th>Frames In Range</th>
<th>Frames In FOV</th>
<th>Result</th>
</tr>
</thead>
<tbody>
{aruco_rows_html}
</tbody>
</table>
</div>
</div>

<div class="panel">
<small>
<b>Current ArUco scoring:</b>
A confirmed detection whose ID exists in the GT map
is counted as TP. Unknown or incorrect IDs are FP.
A GT marker becomes FN only after it has been visible
for the configured minimum number of frames and was
not detected. Position accuracy is evaluated separately
using the configured ArUco position-error threshold.
</small>
</div>
</section>
<section id="system" class="page"><div class="page-title"><div><h2>Full System & TF Health</h2><p>One place for current system status, localization and transform validation.</p></div></div><div class="panel"><div class="table-wrap"><table><thead><tr><th>Check</th><th>Status / Rate</th><th>Details</th></tr></thead><tbody>{tf_rows}</tbody></table></div>{tf_chart}</div><div class="panel"><h3>Current scoring summary</h3><div class="cards">{card_html}</div></div></section>
<div class="footer">The dashboard refreshes every second. The selected tab is preserved after refresh.</div></div>
<script>
function showPage(id,btn){{document.querySelectorAll('.page').forEach(x=>x.classList.remove('active'));document.querySelectorAll('.tab').forEach(x=>x.classList.remove('active'));document.getElementById(id).classList.add('active');btn.classList.add('active');localStorage.setItem('roarTab',id)}}
const saved=localStorage.getItem('roarTab');if(saved){{const page=document.getElementById(saved);const btn=[...document.querySelectorAll('.tab')].find(x=>x.getAttribute('onclick').includes("'"+saved+"'"));if(page&&btn)showPage(saved,btn)}}
</script></body></html>'''


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBenchmarkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.update_outputs()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
