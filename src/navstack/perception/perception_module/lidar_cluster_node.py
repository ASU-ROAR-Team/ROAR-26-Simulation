#!/usr/bin/env python3
"""
LiDAR Cluster Node  (ERC Mars Edition)
=======================================
Pipeline order per frame:
  Stage 0  – ROI pass-through          (GPU-accelerated distance mask)
  Stage 1  – Voxel-grid downsampling
  Stage 2  – SOR  (Statistical Outlier Removal)
  Stage 3  – ROR  (Radius Outlier Removal)
  Stage 4  – Slope-aware iterative RANSAC ground/ramp removal
  Stage 4b – Surface-normal roughness filter (rocks vs smooth slopes)
  Stage 5  – DBSCAN clustering
  Stage 6  – Bounding-box + aspect-ratio + distance + height filters

All thresholds live in config/perception_params.yaml.
GPU note: open3d.core CUDA tensors accelerate the Stage-0 distance mask and
the Stage-4b normal/curvature computation.  Automatically falls back to CPU.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray, Detection3D, BoundingBox3D
from vision_msgs.msg import ObjectHypothesisWithPose
from geometry_msgs.msg import Pose, PoseWithCovariance, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from active_mapping_msgs.msg import ObstacleDetection, ObstacleDetectionArray
import open3d as o3d
import numpy as np

# ── GPU availability detection ──────────────────────────────────────────────
try:
    import open3d.core as o3c
    _CUDA_OK = o3c.cuda.is_available()
except Exception:
    _CUDA_OK = False

# Drop-oldest-frame QoS — never let stale frames pile up
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Gravity vertical in ZED camera frame (Y-axis points DOWN)
_GRAVITY = np.array([0.0, 1.0, 0.0], dtype=np.float64)


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__('lidar_cluster_node')

        # ── Declare all parameters (YAML overrides these defaults) ──────────

        # GPU flag
        self.declare_parameter('use_gpu', True)

        # Input topic
        self.declare_parameter('lidar_topic', '/zed2i/points_updated')

        # Stage 0: ROI
        self.declare_parameter('max_distance', 6.0)

        # Stage 1: Voxel grid
        self.declare_parameter('voxel_size', 0.06)

        # Stage 2: SOR
        self.declare_parameter('sor_nb_neighbors', 25)
        self.declare_parameter('sor_std_ratio', 1.8)
        self.declare_parameter('sor_enabled', True)

        # Stage 3: ROR
        self.declare_parameter('ror_radius', 0.12)
        self.declare_parameter('ror_min_neighbors', 6)
        self.declare_parameter('ror_enabled', True)

        # Stage 4: Slope-aware RANSAC
        self.declare_parameter('ground_distance_threshold', 0.12)
        self.declare_parameter('plane_max_iterations', 4)
        self.declare_parameter('plane_min_inliers', 150)
        self.declare_parameter('ground_max_tilt_deg', 45.0)   # planes tilted > this are NOT removed
        self.declare_parameter('ground_min_z_height', 0.3)    # plane centroid min depth
        self.declare_parameter('ground_max_z_height', 8.0)    # plane centroid max depth

        # Stage 4b: Normal roughness filter
        self.declare_parameter('normal_search_radius', 0.15)
        self.declare_parameter('min_roughness_score', 0.02)
        self.declare_parameter('roughness_filter_enabled', True)

        # Stage 5: DBSCAN
        self.declare_parameter('cluster_tolerance', 0.25)
        self.declare_parameter('min_cluster_size', 15)
        self.declare_parameter('max_cluster_size', 3000)

        # Stage 6: BBox filter
        self.declare_parameter('min_bbox_x', 0.08)
        self.declare_parameter('max_bbox_x', 2.0)
        self.declare_parameter('min_bbox_y', 0.04)
        self.declare_parameter('max_bbox_y', 1.5)
        self.declare_parameter('min_bbox_z', 0.08)
        self.declare_parameter('max_bbox_z', 2.0)

        # Stage 6: Distance + height filter
        self.declare_parameter('min_cluster_distance', 0.4)
        self.declare_parameter('max_cluster_distance', 6.0)
        self.declare_parameter('min_center_y', -0.5)
        self.declare_parameter('max_center_y',  1.5)

        # Stage 6: Aspect-ratio filter (depth/height ratio)
        self.declare_parameter('max_depth_height_ratio', 6.0)

        # ── Determine GPU mode ───────────────────────────────────────────────
        want_gpu = self.get_parameter('use_gpu').value
        self._use_gpu = want_gpu and _CUDA_OK
        if self._use_gpu:
            self._gpu_device = o3c.Device('CUDA:0')
            self.get_logger().info('GPU mode ENABLED  (open3d.core CUDA:0)')
        else:
            self.get_logger().info(
                f'GPU mode DISABLED  (use_gpu={want_gpu}, cuda_ok={_CUDA_OK})'
            )

        # ── Subscriber ───────────────────────────────────────────────────────
        topic = self.get_parameter('lidar_topic').value
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.listener_callback, _SENSOR_QOS
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.cluster_pub     = self.create_publisher(Detection3DArray,    '/detection/lidar_clusters',     10)
        self.raw_cluster_pub = self.create_publisher(Detection3DArray,    '/detection/lidar_clusters_raw', 10)
        self.slam_pub        = self.create_publisher(ObstacleDetectionArray, '/detections/obstacles',      10)
        self.filtered_pub    = self.create_publisher(PointCloud2,         '/lidar/points_filtered',        10)
        self.no_ground_pub   = self.create_publisher(PointCloud2,         '/lidar/points_no_ground',       10)
        self.marker_pub      = self.create_publisher(MarkerArray,         '/visualization/cluster_markers', 10)

        self.get_logger().info('LiDAR Cluster Node started — ERC Mars pipeline active.')

    # ────────────────────────────────────────────────────────────────────────
    # Main callback — runs every incoming point cloud frame
    # ────────────────────────────────────────────────────────────────────────

    def listener_callback(self, msg: PointCloud2):
        # Read all params fresh each call → ros2 param set takes effect live
        p = self._read_params()

        # ── Stage 0: GPU-accelerated ROI distance mask ───────────────────
        points, colors = self._pc2_to_numpy_rgb(msg)
        if points.shape[0] == 0:
            return

        if self._use_gpu:
            # Upload to GPU, compute squared norms, threshold, download mask
            pts_t  = o3c.Tensor(points, dtype=o3c.float32, device=self._gpu_device)
            sq     = (pts_t * pts_t).sum(1)                     # Nx1 squared distances
            mask_t = sq < float(p['max_dist'] ** 2)             # boolean mask on GPU
            mask   = mask_t.cpu().numpy().astype(bool)
        else:
            mask = np.linalg.norm(points, axis=1) < p['max_dist']

        points = points[mask]
        colors = colors[mask]
        if points.shape[0] == 0:
            return

        # Wrap in Open3D legacy PointCloud for the rest of the pipeline
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)

        # ── Stage 1: Voxel-grid downsampling ────────────────────────────
        pcd = pcd.voxel_down_sample(voxel_size=p['voxel_size'])
        if len(pcd.points) < 10:
            return

        # ── Stage 2: Statistical Outlier Removal (SOR) ──────────────────
        # Removes per-point speckle by comparing each point's mean
        # neighbour distance to the global mean±σ across all points.
        if p['sor_en'] and len(pcd.points) >= p['sor_k']:
            pcd, _ = pcd.remove_statistical_outlier(
                nb_neighbors=int(p['sor_k']),
                std_ratio=float(p['sor_std'])
            )
            if len(pcd.points) < 5:
                return

        # ── Stage 3: Radius Outlier Removal (ROR) ───────────────────────
        # Removes isolated points (sensor reflections) that survive SOR.
        if p['ror_en'] and len(pcd.points) >= p['ror_n']:
            pcd, _ = pcd.remove_radius_outlier(
                nb_points=int(p['ror_n']),
                radius=float(p['ror_r'])
            )
            if len(pcd.points) < 5:
                return

        # Publish clean cloud (with ground, for debugging in RViz)
        self.filtered_pub.publish(self._pcd_to_pc2_rgb(pcd, msg.header))

        # ── Stage 4: Slope-aware iterative RANSAC plane removal ─────────
        # Strips ground + ramps while PRESERVING steep rock faces.
        pcd_ng = self._remove_planes_slope_aware(pcd, p)
        if pcd_ng is None or len(pcd_ng.points) == 0:
            return

        # ── Stage 4b: Surface-normal roughness filter ────────────────────
        # Removes any remaining smooth-slope points; keeps only rough/sharp rock points.
        if p['roughness_en']:
            pcd_ng = self._roughness_filter(pcd_ng, p)
            if pcd_ng is None or len(pcd_ng.points) < 5:
                return

        self.no_ground_pub.publish(self._pcd_to_pc2_rgb(pcd_ng, msg.header))

        # ── Stage 5: DBSCAN clustering ───────────────────────────────────
        # Groups surviving points into candidate obstacle clusters.
        labels = np.array(
            pcd_ng.cluster_dbscan(
                eps=p['cluster_tol'],
                min_points=p['min_cluster'],
                print_progress=False
            )
        )

        # Build message containers
        detections     = Detection3DArray(header=msg.header)
        raw_detections = Detection3DArray(header=msg.header)
        slam_detections = ObstacleDetectionArray(header=msg.header)

        max_label = int(labels.max()) if labels.size > 0 else -1

        for i in range(max_label + 1):
            idx = np.where(labels == i)[0]
            if len(idx) < p['min_cluster'] or len(idx) > p['max_cluster']:
                continue  # DBSCAN already filters by min, but max is our extra gate

            cluster_pcd = pcd_ng.select_by_index(idx)
            aabb   = cluster_pcd.get_axis_aligned_bounding_box()
            center = aabb.get_center()
            extent = aabb.get_extent()
            sx, sy, sz = float(extent[0]), float(extent[1]), float(extent[2])
            cx, cy, cz = float(center[0]), float(center[1]), float(center[2])

            # Assemble Detection3D (shared by raw + filtered arrays)
            det = self._make_detection3d(msg.header, center, extent, i)
            raw_detections.detections.append(det)

            # ── Stage 6: Bounding-box shape filter ──────────────────────
            if not (p['min_bx'] <= sx <= p['max_bx'] and
                    p['min_by'] <= sy <= p['max_by'] and
                    p['min_bz'] <= sz <= p['max_bz']):
                self.get_logger().debug(
                    f'Cluster {i} rejected by bbox: x={sx:.2f} y={sy:.2f} z={sz:.2f}'
                )
                continue

            # ── Stage 6: Aspect-ratio filter (flat slab rejection) ───────
            # A ramp remnant is wide+deep but not tall.  Rocks are chunkier.
            if p['max_dr'] > 0 and sy > 1e-3 and (sz / sy) > p['max_dr']:
                self.get_logger().debug(
                    f'Cluster {i} rejected by depth/height={sz/sy:.1f} > {p["max_dr"]}'
                )
                continue

            # ── Stage 6: Distance filter ─────────────────────────────────
            cl_dist = float(np.linalg.norm([cx, cy, cz]))
            if cl_dist < p['min_cdist'] or cl_dist > p['max_cdist']:
                continue

            # ── Stage 6: Centre-height (Y-axis) filter ───────────────────
            # ZED Y=down; positive Y = below camera (on ground level).
            # Rocks sit near Y=0..+0.5; reject clusters above or far below.
            if cy < p['min_cy'] or cy > p['max_cy']:
                continue

            detections.detections.append(det)

            # Build SLAM-format obstacle
            slam_obs = ObstacleDetection()
            slam_obs.position   = Point(x=cx, y=cy, z=cz)
            slam_obs.dimensions = Vector3(x=sx, y=sy, z=sz)
            slam_detections.detections.append(slam_obs)

        # Publish
        self.raw_cluster_pub.publish(raw_detections)
        self.cluster_pub.publish(detections)
        self.slam_pub.publish(slam_detections)
        self._publish_markers(detections, msg.header)

    # ────────────────────────────────────────────────────────────────────────
    # Parameter snapshot helper — call once per callback
    # ────────────────────────────────────────────────────────────────────────

    def _read_params(self) -> dict:
        g = self.get_parameter
        return dict(
            max_dist    = g('max_distance').value,
            voxel_size  = g('voxel_size').value,
            sor_k       = g('sor_nb_neighbors').value,
            sor_std     = g('sor_std_ratio').value,
            sor_en      = g('sor_enabled').value,
            ror_r       = g('ror_radius').value,
            ror_n       = g('ror_min_neighbors').value,
            ror_en      = g('ror_enabled').value,
            gnd_thresh  = g('ground_distance_threshold').value,
            plane_iters = g('plane_max_iterations').value,
            plane_inliers = g('plane_min_inliers').value,
            gnd_tilt    = g('ground_max_tilt_deg').value,
            gnd_min_z   = g('ground_min_z_height').value,
            gnd_max_z   = g('ground_max_z_height').value,
            norm_radius = g('normal_search_radius').value,
            roughness   = g('min_roughness_score').value,
            roughness_en = g('roughness_filter_enabled').value,
            cluster_tol = g('cluster_tolerance').value,
            min_cluster = g('min_cluster_size').value,
            max_cluster = g('max_cluster_size').value,
            min_bx = g('min_bbox_x').value, max_bx = g('max_bbox_x').value,
            min_by = g('min_bbox_y').value, max_by = g('max_bbox_y').value,
            min_bz = g('min_bbox_z').value, max_bz = g('max_bbox_z').value,
            min_cdist = g('min_cluster_distance').value,
            max_cdist = g('max_cluster_distance').value,
            min_cy    = g('min_center_y').value,
            max_cy    = g('max_center_y').value,
            max_dr    = g('max_depth_height_ratio').value,
        )

    # ────────────────────────────────────────────────────────────────────────
    # Stage 4: Slope-aware iterative RANSAC plane removal
    # ────────────────────────────────────────────────────────────────────────

    def _remove_planes_slope_aware(self, pcd, p) -> o3d.geometry.PointCloud:
        """
        Iteratively finds and removes dominant planes, but ONLY if the plane:
          (a) has >= plane_min_inliers points (small planes = rock faces → keep)
          (b) its normal tilts <= ground_max_tilt_deg from vertical (ZED Y axis)
              → near-vertical walls / rock faces are NEVER removed
          (c) its centroid depth (Z) is within [gnd_min_z, gnd_max_z]
              → prevents accidentally removing a floating rock cluster

        This is the key improvement over the old all-planes-stripped approach:
        a ramp at 30° is removed; a near-vertical boulder face at 80° is kept.
        """
        cos_limit = math.cos(math.radians(float(p['gnd_tilt'])))
        current   = pcd

        for i in range(int(p['plane_iters'])):
            if len(current.points) < 20:
                break

            try:
                plane_model, inliers = current.segment_plane(
                    distance_threshold=float(p['gnd_thresh']),
                    ransac_n=3,
                    num_iterations=200
                )
            except Exception:
                break

            if len(inliers) < int(p['plane_inliers']):
                # Too few inliers → this is a small feature (rock), not ground. Stop.
                self.get_logger().debug(
                    f'RANSAC iter {i}: {len(inliers)} inliers < {p["plane_inliers"]}, stopping.'
                )
                break

            # Compute cosine between plane normal and gravity (Y axis)
            normal = np.array(plane_model[:3], dtype=np.float64)
            normal /= np.linalg.norm(normal) + 1e-9
            cos_with_gravity = abs(float(np.dot(normal, _GRAVITY)))

            # Compute centroid depth (Z = forward in ZED frame)
            pts_arr = np.asarray(current.points)
            centroid_z = float(np.mean(pts_arr[inliers, 2]))

            # Check centroid depth is in valid ground range
            depth_ok = p['gnd_min_z'] <= centroid_z <= p['gnd_max_z']

            # Remove the plane only if it is "ground-like"
            if cos_with_gravity >= cos_limit and depth_ok:
                axis = ['X', 'Y', 'Z'][int(np.argmax(np.abs(normal)))]
                self.get_logger().debug(
                    f'RANSAC iter {i}: removed {len(inliers)} pts  '
                    f'normal≈{normal.round(2)}  dominant={axis}  '
                    f'tilt_ok  centroid_z={centroid_z:.2f}'
                )
                current = current.select_by_index(inliers, invert=True)
            else:
                # This plane is too steep (rock face) or at wrong depth — stop.
                self.get_logger().debug(
                    f'RANSAC iter {i}: KEPT {len(inliers)} pts  '
                    f'cos_grav={cos_with_gravity:.2f} < {cos_limit:.2f} '
                    f'or depth_ok={depth_ok}. Stopping.'
                )
                break

        return current

    # ────────────────────────────────────────────────────────────────────────
    # Stage 4b: Surface-normal roughness filter
    # ────────────────────────────────────────────────────────────────────────

    def _roughness_filter(self, pcd, p) -> o3d.geometry.PointCloud:
        """
        Computes surface normals for every point and measures local roughness
        (curvature / normal variance) in the neighbourhood.

        Algorithm:
          1. Estimate normals using PCA in a sphere of radius normal_search_radius.
          2. For each point i, gather its neighbours within the same radius.
          3. Compute the mean normal of those neighbours.
          4. Roughness score = mean angular deviation of neighbours from the mean normal.
             High score → points are on a sharp edge or irregular surface (ROCK).
             Low score  → points are on a flat/smooth surface (SLOPE/GROUND).
          5. Drop points with roughness < min_roughness_score.

        GPU acceleration: step 1 (normal estimation) uses open3d.core CUDA
        tensors when use_gpu=true; steps 2-5 use NumPy on CPU (normal variance
        computation is memory-bound, not compute-bound, so CPU is adequate).
        """
        if len(pcd.points) < 10:
            return pcd

        radius = float(p['norm_radius'])
        thresh = float(p['roughness'])

        if self._use_gpu:
            # Estimate normals on GPU via the Tensor API, then bring back to CPU
            try:
                pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcd)
                pcd_t = pcd_t.to(self._gpu_device)
                pcd_t.estimate_normals(
                    o3d.core.SizeVector([0, 30]),          # min 0, max 30 neighbours
                    radius=radius
                )
                normals = pcd_t.point.normals.cpu().numpy()
            except Exception:
                # GPU path failed (geometry API unstable) → fall back to CPU
                pcd.estimate_normals(
                    search_param=o3d.geometry.KDTreeSearchParamRadius(radius)
                )
                normals = np.asarray(pcd.normals)
        else:
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamRadius(radius)
            )
            normals = np.asarray(pcd.normals)

        if normals.shape[0] == 0:
            return pcd

        # Normalise (PCA can return non-unit normals in edge cases)
        norms_len = np.linalg.norm(normals, axis=1, keepdims=True) + 1e-9
        normals  /= norms_len

        # Build KD-tree on CPU for neighbourhood queries
        pts_arr = np.asarray(pcd.points)
        kdtree  = o3d.geometry.KDTreeFlann(pcd)

        roughness_scores = np.zeros(len(pts_arr), dtype=np.float32)

        for j in range(len(pts_arr)):
            [k, nb_idx, _] = kdtree.search_radius_vector_3d(pts_arr[j], radius)
            if k < 3:
                # Too few neighbours → treat as isolated point, keep it
                roughness_scores[j] = 1.0
                continue
            nb_normals  = normals[nb_idx, :]
            mean_normal = nb_normals.mean(axis=0)
            mn_len      = np.linalg.norm(mean_normal) + 1e-9
            mean_normal /= mn_len
            # Dot product of each neighbour normal with the mean normal
            dots         = np.clip(nb_normals @ mean_normal, -1.0, 1.0)
            # Angular variance: 1 - mean(|dot|) = 0 for flat, ~1 for chaotic
            roughness_scores[j] = float(1.0 - np.mean(np.abs(dots)))

        # Keep only points that are rough enough to be obstacles
        keep_mask = roughness_scores >= thresh
        if keep_mask.sum() == 0:
            return pcd  # fallback: keep all (avoid empty cloud)

        return pcd.select_by_index(np.where(keep_mask)[0])

    # ────────────────────────────────────────────────────────────────────────
    # Helper: build Detection3D message for a cluster
    # ────────────────────────────────────────────────────────────────────────

    def _make_detection3d(self, header, center, extent, cluster_id: int) -> Detection3D:
        det  = Detection3D()
        det.header = header
        hyp  = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = f'rock_{cluster_id}'
        hyp.hypothesis.score    = 1.0
        hyp.pose = PoseWithCovariance()
        hyp.pose.pose = Pose()
        hyp.pose.pose.position.x = float(center[0])
        hyp.pose.pose.position.y = float(center[1])
        hyp.pose.pose.position.z = float(center[2])
        hyp.pose.pose.orientation.w = 1.0
        det.results.append(hyp)
        bbox = BoundingBox3D()
        bbox.center.position.x = float(center[0])
        bbox.center.position.y = float(center[1])
        bbox.center.position.z = float(center[2])
        bbox.size.x = float(extent[0])
        bbox.size.y = float(extent[1])
        bbox.size.z = float(extent[2])
        det.bbox = bbox
        return det

    # ────────────────────────────────────────────────────────────────────────
    # RViz marker builder
    # ────────────────────────────────────────────────────────────────────────

    COLOURS = [
        ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.4),  # cyan
        ColorRGBA(r=1.0, g=0.4, b=0.0, a=0.4),  # orange
        ColorRGBA(r=0.4, g=1.0, b=0.0, a=0.4),  # lime
        ColorRGBA(r=1.0, g=0.0, b=0.6, a=0.4),  # pink
        ColorRGBA(r=0.6, g=0.0, b=1.0, a=0.4),  # purple
        ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.4),  # yellow
    ]

    def _publish_markers(self, detections: Detection3DArray, header):
        ma = MarkerArray()
        del_m = Marker(); del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)

        for idx, det in enumerate(detections.detections):
            colour = self.COLOURS[idx % len(self.COLOURS)]
            pos    = det.bbox.center.position

            # Filled cube
            cube = Marker()
            cube.header = header; cube.ns = 'rock_box'; cube.id = idx * 3
            cube.type = Marker.CUBE; cube.action = Marker.ADD
            cube.pose.position = pos; cube.pose.orientation.w = 1.0
            cube.scale.x = max(det.bbox.size.x, 0.05)
            cube.scale.y = max(det.bbox.size.y, 0.05)
            cube.scale.z = max(det.bbox.size.z, 0.05)
            cube.color = colour; cube.lifetime.nanosec = 500_000_000
            ma.markers.append(cube)

            # Wireframe outline
            wire = Marker()
            wire.header = header; wire.ns = 'rock_wire'; wire.id = idx * 3 + 1
            wire.type = Marker.LINE_LIST; wire.action = Marker.ADD
            wire.pose.position = pos; wire.pose.orientation.w = 1.0
            wire.scale.x = 0.02
            wire.color = ColorRGBA(r=colour.r, g=colour.g, b=colour.b, a=1.0)
            wire.lifetime.nanosec = 500_000_000
            hx, hy, hz = det.bbox.size.x/2, det.bbox.size.y/2, det.bbox.size.z/2
            corners = [
                Point(x= hx, y= hy, z= hz), Point(x=-hx, y= hy, z= hz),
                Point(x= hx, y=-hy, z= hz), Point(x=-hx, y=-hy, z= hz),
                Point(x= hx, y= hy, z=-hz), Point(x=-hx, y= hy, z=-hz),
                Point(x= hx, y=-hy, z=-hz), Point(x=-hx, y=-hy, z=-hz),
            ]
            for a, b in [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]:
                wire.points.append(corners[a]); wire.points.append(corners[b])
            ma.markers.append(wire)

            # Distance label
            text = Marker()
            text.header = header; text.ns = 'rock_label'; text.id = idx * 3 + 2
            text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD
            text.pose.position.x = pos.x
            text.pose.position.y = pos.y
            text.pose.position.z = pos.z + det.bbox.size.z / 2.0 + 0.12
            text.pose.orientation.w = 1.0; text.scale.z = 0.12
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            dist = float(np.linalg.norm([pos.x, pos.y, pos.z]))
            text.text = f'{dist:.1f} m'; text.lifetime.nanosec = 500_000_000
            ma.markers.append(text)

        self.marker_pub.publish(ma)

    # ────────────────────────────────────────────────────────────────────────
    # I/O helpers
    # ────────────────────────────────────────────────────────────────────────

    def _pc2_to_numpy_rgb(self, msg: PointCloud2):
        """Parse XYZ (+ optional packed RGB float) from a PointCloud2 message."""
        available = {f.name for f in msg.fields}
        has_rgb   = 'rgb' in available
        if has_rgb:
            gen   = point_cloud2.read_points(msg, field_names=('x','y','z','rgb'), skip_nans=True)
            dtype = [('x',np.float32),('y',np.float32),('z',np.float32),('rgb',np.float32)]
        else:
            gen   = point_cloud2.read_points(msg, field_names=('x','y','z'), skip_nans=True)
            dtype = [('x',np.float32),('y',np.float32),('z',np.float32)]

        arr = np.fromiter(gen, dtype=dtype)
        if arr.size == 0:
            return np.zeros((0,3),np.float32), np.zeros((0,3),np.float64)

        xyz   = np.stack([arr['x'], arr['y'], arr['z']], axis=1)
        valid = np.isfinite(xyz).all(axis=1)
        xyz   = xyz[valid].astype(np.float32)

        if has_rgb:
            rgb_u32 = arr['rgb'][valid].view(np.uint32)
            r = ((rgb_u32 >> 16) & 0xFF).astype(np.float64) / 255.0
            g = ((rgb_u32 >>  8) & 0xFF).astype(np.float64) / 255.0
            b = ( rgb_u32        & 0xFF).astype(np.float64) / 255.0
            colors = np.stack([r, g, b], axis=1)
        else:
            colors = np.zeros((len(xyz), 3), dtype=np.float64)
        return xyz, colors

    def _pcd_to_pc2_rgb(self, pcd: o3d.geometry.PointCloud, header) -> PointCloud2:
        """Convert an Open3D PointCloud to a sensor_msgs/PointCloud2 (XYZRGB)."""
        pts = np.asarray(pcd.points).astype(np.float32)
        n   = len(pts)
        if n == 0:
            return point_cloud2.create_cloud_xyz32(header, pts)
        if len(pcd.colors) == n:
            cols    = (np.asarray(pcd.colors) * 255).astype(np.uint32)
            rgb_u32 = (cols[:,0] << 16) | (cols[:,1] << 8) | cols[:,2]
            rgb_f32 = rgb_u32.view(np.float32)
        else:
            rgb_f32 = np.zeros(n, dtype=np.float32)
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        buf = np.zeros(n, dtype=[('x',np.float32),('y',np.float32),
                                  ('z',np.float32),('rgb',np.float32)])
        buf['x'] = pts[:,0]; buf['y'] = pts[:,1]
        buf['z'] = pts[:,2]; buf['rgb'] = rgb_f32
        out = PointCloud2()
        out.header = header; out.height = 1; out.width = n
        out.fields = fields; out.is_bigendian = False
        out.point_step = 16; out.row_step = 16 * n
        out.data = buf.tobytes(); out.is_dense = True
        return out


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
