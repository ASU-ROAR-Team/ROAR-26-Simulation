from __future__ import annotations

import os
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header, Float32MultiArray
import tf2_ros
import tf2_geometry_msgs

from active_mapping_msgs.msg import ObstacleDetectionArray


class _ObstacleTrack:
    """A tracked obstacle: filtered (x, y, z) bbox-center position with
    per-axis variance, refined by a Kalman update on every associated
    sighting. (Static motion model + identity measurement model, so the
    'EKF' reduces to a plain per-axis scalar Kalman filter.)"""
    __slots__ = (
        "x", "y", "z", "var_x", "var_y", "var_z",
        "dim_x", "dim_y", "dim_z", "class_id", "hits",
    )

    def __init__(self, x, y, z, var_x, var_y, var_z,
                 dim_x, dim_y, dim_z, class_id):
        self.x = x
        self.y = y
        self.z = z
        self.var_x = var_x
        self.var_y = var_y
        self.var_z = var_z
        self.dim_x = dim_x
        self.dim_y = dim_y
        self.dim_z = dim_z
        self.class_id = class_id
        self.hits = 1

    def mahalanobis_sq(self, mx, my, mz, r_xy, r_z) -> float:
        """Squared Mahalanobis distance of a measurement to this track,
        under innovation covariance P + R per axis."""
        return (
            (mx - self.x) ** 2 / (self.var_x + r_xy)
            + (my - self.y) ** 2 / (self.var_y + r_xy)
            + (mz - self.z) ** 2 / (self.var_z + r_z)
        )

    def update(self, mx, my, mz, dim_x, dim_y, dim_z, r_xy, r_z):
        """Kalman-update the position from a new sighting; dims tracked
        as a running mean over sightings."""
        for axis, meas, r in (("x", mx, r_xy), ("y", my, r_xy), ("z", mz, r_z)):
            state = getattr(self, axis)
            var = getattr(self, "var_" + axis)
            k = var / (var + r)
            setattr(self, axis, state + k * (meas - state))
            setattr(self, "var_" + axis, (1.0 - k) * var)

        self.hits += 1
        n = self.hits
        self.dim_x += (dim_x - self.dim_x) / n
        self.dim_y += (dim_y - self.dim_y) / n
        self.dim_z += (dim_z - self.dim_z) / n

    def mean_variance(self) -> float:
        return (self.var_x + self.var_y + self.var_z) / 3.0


# Main node
class TerrainMappingNode(Node):
    """
    The stack's mapping node: loads a static height map once (produced
    offline, e.g. by the SLAM offline tools) as a trusted, immutable base,
    then verifies live obstacle detections against it with per-obstacle
    Kalman tracks. A detection only reaches the published maps once its
    track is verified: it must be terrain-consistent (bbox bottom sitting on
    the low-covariance base map) and re-sighted until its position variance
    drops below commit_variance_threshold. Verified obstacles are stamped
    on top of the base map; the base terrain itself is never modified.
    """

    def __init__(self):
        super().__init__("terrain_mapping")

        self._declare_parameters()
        self._read_parameters()

        self._init_height_map()
        self._tracks: List[_ObstacleTrack] = []

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(
            ObstacleDetectionArray,
            self._obstacle_topic,
            self._on_detections,
            10,
        )
        self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )

        # Publishers
        latching_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub_occupancy = self.create_publisher(
            OccupancyGrid, "/map", latching_qos
        )
        self._pub_pointcloud = self.create_publisher(
            PointCloud2, "/active_map/pointcloud", latching_qos
        )
        self._pub_heightmap = self.create_publisher(
            OccupancyGrid, "/active_map/heightmap", latching_qos
        )
        # Paired with /active_map/heightmap: [h_min, h_range] of the current
        # publish, since the grid above is min-max normalized to [0, 100] over
        # ITS OWN observed range (which varies per heightmap file) rather than
        # a fixed physical scale. Consumers need this to decode cell values
        # back into meters correctly.
        self._pub_heightmap_range = self.create_publisher(
            Float32MultiArray, "/active_map/heightmap_range", latching_qos
        )

        # Publish timer.
        self._publish_timer = self.create_timer(
            1.0 / self._publish_rate, self._publish_maps
        )

        self.get_logger().info(
            f"TerrainMappingNode ready — static grid {self._rows}x{self._cols} "
            f"@ {self._resolution:.3f} m/cell, origin "
            f"({self._origin_x:.2f}, {self._origin_y:.2f}) "
            f"loaded from '{self._heightmap_file}'"
        )

    # Parameter helpers

    def _declare_parameters(self):
        def p(name, default, desc=""):
            self.declare_parameter(name, default, ParameterDescriptor(description=desc))

        p("map_resolution",  0.1,   "Fallback metres per grid cell, used if heightmap_file is empty/missing")
        p("map_width",      100.0,  "Fallback map width in metres, used if heightmap_file is empty/missing")
        p("map_height",     100.0,  "Fallback map height in metres, used if heightmap_file is empty/missing")
        p("map_origin_x",   -50.0,  "Fallback world-frame X of grid origin")
        p("map_origin_y",   -50.0,  "Fallback world-frame Y of grid origin")
        p("heightmap_file",  "",    "Absolute path to a static .npz height map "
                                    "(keys: grid, resolution, origin_x, origin_y)")
        p("aruco_map_file",  "",    "Absolute path to the ArUco marker world-position YAML "
                                    "(schema: markers: [{id, position: [x, y, z]}]) to stamp "
                                    "as static obstacles into the base height map")
        p("aruco_obstacle_height", 1.5,
          "Height [m] above terrain stamped at each ArUco marker's starting (x, y) cell")
        p("meas_variance_xy",  0.04, "Detection position measurement variance, X/Y [m^2]")
        p("meas_variance_z",   0.04, "Detection position measurement variance, Z [m^2]")
        p("initial_height_variance", 1e-4,
          "Variance of the trusted base height map [m^2] — small = high confidence")
        p("unknown_height_variance", 1e4,
          "Variance assumed for unmapped (NaN) base cells [m^2]")
        p("association_gate", 3.0,  "Mahalanobis gate for matching a detection to an existing track [sigma]")
        p("terrain_gate",     3.0,  "Gate for a new obstacle's bbox bottom vs trusted terrain height [sigma]")
        p("commit_variance_threshold", 0.025,
          "Track is verified (stamped into the maps) once its mean position variance drops below this [m^2]")
        p("occupancy_threshold", 0.45, "Height above base to mark cell occupied [m]")
        p("obstacle_topic",  "/detections/obstacles", "Obstacle detection topic")
        p("odom_topic",      "/odom",                 "Odometry topic")
        p("map_frame",       "map", "Fixed world frame id")
        p("publish_rate",    2.0,   "Map publish rate [Hz]")

    def _read_parameters(self):
        g = self.get_parameter
        self._resolution    = g("map_resolution").value
        self._map_width     = g("map_width").value
        self._map_height    = g("map_height").value
        self._origin_x      = g("map_origin_x").value
        self._origin_y      = g("map_origin_y").value
        self._heightmap_file = g("heightmap_file").value
        self._aruco_map_file = g("aruco_map_file").value
        self._aruco_obstacle_height = g("aruco_obstacle_height").value
        self._r_xy          = g("meas_variance_xy").value
        self._r_z           = g("meas_variance_z").value
        self._base_variance = g("initial_height_variance").value
        self._unknown_variance = g("unknown_height_variance").value
        self._assoc_gate_sq = g("association_gate").value ** 2
        self._terrain_gate_sq = g("terrain_gate").value ** 2
        self._commit_variance = g("commit_variance_threshold").value
        self._occ_threshold = g("occupancy_threshold").value
        self._obstacle_topic = g("obstacle_topic").value
        self._odom_topic    = g("odom_topic").value
        self._map_frame     = g("map_frame").value
        self._publish_rate  = g("publish_rate").value

        self._cols = int(round(self._map_width  / self._resolution))
        self._rows = int(round(self._map_height / self._resolution))

    # Height map initialisation

    def _init_height_map(self):
        """Load the static height map from heightmap_file (.npz) or fall
        back to a flat zero map. The .npz's own grid/resolution/origin take
        priority over the map_* fallback parameters, since they describe the
        exact geometry the offline tool baked the map with."""
        loaded = self._load_npz(self._heightmap_file) if self._heightmap_file else None

        if loaded is not None:
            base = loaded
            self.get_logger().info(f"Loaded static height map from {self._heightmap_file}")
        else:
            if self._heightmap_file:
                self.get_logger().warn(
                    f"heightmap_file '{self._heightmap_file}' not found or invalid — using flat map."
                )
            base = np.zeros((self._rows, self._cols), dtype=np.float32)

        # The base map is the trusted prior (initial_height_variance) and is
        # immutable: verified obstacle tracks are stamped ON TOP of it into
        # self._height_map by _rebuild_height_map(), never blended into it.
        self._base_height_map = base
        # ArUco markers are a priori known landmarks, not live detections —
        # bake them into the trusted base itself (before it's ever copied)
        # so they survive every subsequent _rebuild_height_map() cycle.
        self._stamp_aruco_obstacles()
        self._height_map = self._base_height_map.copy()

    def _stamp_aruco_obstacles(self):
        """Stamp each ArUco marker's starting (x, y) cell into the base
        height map as a static obstacle, aruco_obstacle_height metres above
        whatever terrain height is already there. Markers are known ahead of
        time (unlike live detections), so they go straight into the trusted
        base rather than through the Kalman-tracked obstacle pipeline."""
        if not self._aruco_map_file:
            return

        if not os.path.isfile(self._aruco_map_file):
            self.get_logger().warn(
                f"aruco_map_file '{self._aruco_map_file}' not found — skipping marker obstacles."
            )
            return

        try:
            with open(self._aruco_map_file, "r") as f:
                data = yaml.safe_load(f) or {}
            markers = data.get("markers", [])
        except Exception as exc:
            self.get_logger().warn(f"Failed to load aruco_map_file '{self._aruco_map_file}': {exc}")
            return

        stamped = 0
        for marker in markers:
            wx, wy, _wz = marker["position"]
            col = int((wx - self._origin_x) / self._resolution)
            row = int((wy - self._origin_y) / self._resolution)
            if not (0 <= row < self._rows and 0 <= col < self._cols):
                continue

            terrain = self._base_height_map[row, col]
            terrain_z = 0.0 if np.isnan(terrain) else float(terrain)
            self._base_height_map[row, col] = terrain_z + self._aruco_obstacle_height
            stamped += 1

        self.get_logger().info(
            f"Stamped {stamped}/{len(markers)} ArUco marker obstacles from "
            f"'{self._aruco_map_file}' (+{self._aruco_obstacle_height:.2f} m above terrain)"
        )

    def _load_npz(self, path: str) -> Optional[np.ndarray]:
        if not os.path.isfile(path):
            return None
        try:
            data = np.load(path)
            grid = np.asarray(data["grid"], dtype=np.float32)
        except Exception as exc:
            self.get_logger().warn(f"Failed to load heightmap_file '{path}': {exc}")
            return None

        self._rows, self._cols = grid.shape
        if "resolution" in data:
            self._resolution = float(data["resolution"])
        if "origin_x" in data:
            self._origin_x = float(data["origin_x"])
        if "origin_y" in data:
            self._origin_y = float(data["origin_y"])

        return grid

    # Odometry callback (stored for fallback transform when TF is not available)

    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    # Detection callback

    def _on_detections(self, msg: ObstacleDetectionArray):
        updated = 0
        created = 0
        rejected = 0

        for det in msg.detections:
            world_pt = self._transform_to_map_frame(det.position, msg.header)
            if world_pt is None:
                continue  # transform failed — skip this detection

            wx, wy, wz = world_pt

            # 1. Associate with the nearest existing track (Mahalanobis gate).
            best = None
            best_d_sq = self._assoc_gate_sq
            for track in self._tracks:
                d_sq = track.mahalanobis_sq(wx, wy, wz, self._r_xy, self._r_z)
                if d_sq < best_d_sq:
                    best_d_sq = d_sq
                    best = track

            if best is not None:
                best.update(
                    wx, wy, wz,
                    det.dimensions.x, det.dimensions.y, det.dimensions.z,
                    self._r_xy, self._r_z,
                )
                updated += 1
                continue

            # 2. New obstacle — verify it against the trusted terrain before
            #    a track is born: its bbox bottom must sit on the base map.
            if not self._terrain_consistent(wx, wy, wz - det.dimensions.z / 2.0):
                rejected += 1
                continue

            self._tracks.append(
                _ObstacleTrack(
                    x=wx, y=wy, z=wz,
                    var_x=self._r_xy, var_y=self._r_xy, var_z=self._r_z,
                    dim_x=det.dimensions.x,
                    dim_y=det.dimensions.y,
                    dim_z=det.dimensions.z,
                    class_id=det.class_id,
                )
            )
            created += 1

        if updated or created:
            self._rebuild_height_map()

        if updated or created or rejected:
            self.get_logger().debug(
                f"Detections — track updates: {updated}  new tracks: {created}  "
                f"terrain-rejected: {rejected}  tracks total: {len(self._tracks)}"
            )

    def _terrain_consistent(self, wx: float, wy: float, bottom_z: float) -> bool:
        """Gate a prospective obstacle's bbox bottom against the trusted base
        terrain height at (wx, wy). Base cells carry initial_height_variance
        (tiny — the map is respected); unmapped NaN cells carry
        unknown_height_variance, which makes the gate pass automatically
        there. Out-of-grid detections are likewise unverifiable but allowed."""
        col = int((wx - self._origin_x) / self._resolution)
        row = int((wy - self._origin_y) / self._resolution)
        if not (0 <= row < self._rows and 0 <= col < self._cols):
            return True

        terrain = float(self._base_height_map[row, col])
        var_terrain = self._unknown_variance if np.isnan(terrain) else self._base_variance
        if np.isnan(terrain):
            terrain = 0.0

        innovation_sq = (bottom_z - terrain) ** 2
        return innovation_sq / (var_terrain + self._r_z) < self._terrain_gate_sq

    def _rebuild_height_map(self):
        """Recompute the published height map: the immutable base terrain
        plus every VERIFIED track's bbox top stamped over its footprint at
        the track's current filtered position. Rebuilding from scratch means
        a track whose estimate shifts cells between sightings never leaves a
        stale stamp behind."""
        hm = self._base_height_map.copy()
        for track in self._tracks:
            if track.mean_variance() > self._commit_variance:
                continue  # not yet verified — stays out of the published maps
            top = track.z + track.dim_z / 2.0
            for row, col in self._bbox_to_cells(track.x, track.y, track.dim_x, track.dim_y):
                hm[row, col] = np.fmax(hm[row, col], top)
        self._height_map = hm

    # Geometry helpers

    def _transform_to_map_frame(self, position: Point, header: Header):
        """
        Return (x, y, z) in map frame, or None if the transform is unavailable.
        Wraps tf2 so the rest of the code stays clean.
        """
        source_frame = header.frame_id or self._map_frame

        if source_frame == self._map_frame:
            return position.x, position.y, position.z

        try:
            stamped = PointStamped()
            stamped.header = header
            stamped.point = position
            transformed = self._tf_buffer.transform(
                stamped, self._map_frame, timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            self.get_logger().warn(
                f"TF transform {source_frame}→{self._map_frame} failed: {exc}",
                throttle_duration_sec=5.0,
            )
            return None

    def _bbox_to_cells(
        self, cx: float, cy: float, dim_x: float, dim_y: float
    ) -> List[Tuple[int, int]]:
        """
        Return all grid cells covered by the axis-aligned bounding box footprint.
        The footprint is the XY extent of the bbox centred at (cx, cy).
        """
        half_x = max(dim_x / 2.0, self._resolution / 2.0)
        half_y = max(dim_y / 2.0, self._resolution / 2.0)

        x_min = cx - half_x
        x_max = cx + half_x
        y_min = cy - half_y
        y_max = cy + half_y

        col_min = max(0, int((x_min - self._origin_x) / self._resolution))
        col_max = min(self._cols - 1, int((x_max - self._origin_x) / self._resolution))
        row_min = max(0, int((y_min - self._origin_y) / self._resolution))
        row_max = min(self._rows - 1, int((y_max - self._origin_y) / self._resolution))

        cells = []
        for r in range(row_min, row_max + 1):
            for c in range(col_min, col_max + 1):
                cells.append((r, c))
        return cells

    # Publishing

    def _publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        self._publish_occupancy_grid(stamp)
        self._publish_pointcloud(stamp)
        self._publish_heightmap(stamp)

    def _publish_occupancy_grid(self, stamp):
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._map_frame

        grid.info.map_load_time = stamp
        grid.info.resolution = self._resolution
        grid.info.width  = self._cols
        grid.info.height = self._rows
        grid.info.origin.position.x = self._origin_x
        grid.info.origin.position.y = self._origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Occupied where height is significantly above local ground level.
        # base_height_map's NaN cells (never scanned by the offline tool) are
        # treated as 0 here only so a verified obstacle stamped on one of
        # them can register as occupied — cells no track ever touches stay
        # NaN in height_map too, so the "unmapped" mask below still wins.
        above_base = self._height_map - np.nan_to_num(self._base_height_map, nan=0.0)
        occupied = above_base > self._occ_threshold

        # Cells the offline tool never scanned come through as NaN — mark
        # those unknown (-1) rather than silently calling them free.
        unmapped = np.isnan(self._height_map)

        flat = np.where(occupied, 100, 0).astype(np.int8)
        flat[unmapped] = -1
        grid.data = flat.flatten().tolist()

        self._pub_occupancy.publish(grid)

    def _publish_pointcloud(self, stamp):
        """Emit every grid cell as an (x, y, z=height) point."""
        cols_idx, rows_idx = np.meshgrid(
            np.arange(self._cols), np.arange(self._rows)
        )
        xs = (cols_idx * self._resolution + self._origin_x + self._resolution / 2.0).astype(np.float32)
        ys = (rows_idx * self._resolution + self._origin_y + self._resolution / 2.0).astype(np.float32)
        zs = self._height_map  # float32

        points = np.column_stack(
            [xs.ravel(), ys.ravel(), zs.ravel()]
        )

        header = Header()
        header.stamp = stamp
        header.frame_id = self._map_frame

        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self._pub_pointcloud.publish(cloud_msg)

    def _publish_heightmap(self, stamp):
        """Publish the height map as an OccupancyGrid, min-max normalized to
        [0, 100] over the map's valid (non-NaN) cells. Unscanned/NaN cells
        are marked unknown (-1), matching the semantics of _publish_occupancy_grid."""
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = self._map_frame

        grid.info.map_load_time = stamp
        grid.info.resolution = self._resolution
        grid.info.width  = self._cols
        grid.info.height = self._rows
        grid.info.origin.position.x = self._origin_x
        grid.info.origin.position.y = self._origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        unmapped = np.isnan(self._height_map)
        valid = self._height_map[~unmapped]
        h_min = float(valid.min()) if valid.size > 0 else 0.0
        h_range = float(valid.max()) - h_min if valid.size > 0 else 0.0

        if h_range > 1e-6:
            normalized = (self._height_map - h_min) / h_range * 100.0
        else:
            normalized = np.zeros_like(self._height_map)

        flat = np.clip(np.nan_to_num(normalized, nan=0.0), 0, 100).astype(np.int8)
        flat[unmapped] = -1
        grid.data = flat.flatten().tolist()

        self._pub_heightmap.publish(grid)
        self._pub_heightmap_range.publish(Float32MultiArray(data=[h_min, h_range]))


# Entry point

def main(args=None):
    rclpy.init(args=args)
    node = TerrainMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
