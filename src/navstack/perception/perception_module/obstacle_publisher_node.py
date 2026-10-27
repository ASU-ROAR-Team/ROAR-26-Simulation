# Maintains a persistent map of obstacles in world coordinates.
# Reads LiDAR clusters directly from /detection/lidar_clusters (no YOLO fusion),
# transforms them into the map frame using the ZED's pose, accumulates them
# over time, and continuously publishes the full obstacle array to
# /planning/obstacles so the path planner always has the latest state.

import time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray, Detection3D
from vision_msgs.msg import ObjectHypothesisWithPose

import tf2_ros
import tf2_geometry_msgs


class ObstaclePublisherNode(Node):
    def __init__(self):
        super().__init__('obstacle_publisher_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('source_frame',             'zed_left_camera_frame')
        self.declare_parameter('world_frame',              'odom')   # target TF frame for the obstacle map
        self.declare_parameter('max_obstacle_base_height', 3.0)
        self.declare_parameter('max_detection_distance',   2.5)  # metres — ignore far objects, but keep >= QSTAR for smooth APF
        self.declare_parameter('min_confidence',           0.60) # increased to reject noisy ghost obstacles
        self.declare_parameter('merge_radius',             1.0)
        self.declare_parameter('forget_timeout',           10.0)
        self.declare_parameter('publish_rate',             2.0)
        self.declare_parameter('max_obstacles',            50)
        self.declare_parameter('ema_alpha',                0.2)  # EMA factor: 0.2 means 20% new detection, 80% history (smooths jitter)

        # ── Persistent obstacle store ─────────────────────────────────────────
        # Each entry: {'detection': Detection3D, 'last_seen': float, 'color_idx': int}
        # color_idx is fixed at creation so the box colour never changes
        self.obstacle_map: dict[int, dict] = {}
        self._next_id = 0
        self.latest_stamp = None

        # ── TF2 Listener (Replaces manual pose hacking) ───────────────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── Detection subscriber ──────────────────────────────────────────────
        # Subscribe directly to LiDAR clusters — no YOLO camera fusion needed
        self.sub = self.create_subscription(
            Detection3DArray, '/detection/lidar_clusters', self.callback, 10)

        # ── Clear-map subscriber ──────────────────────────────────────────────
        # Publish True to /planning/clear_obstacles to wipe the whole map
        self.clear_sub = self.create_subscription(
            Bool, '/planning/clear_obstacles', self._clear_callback, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.obstacle_pub = self.create_publisher(Detection3DArray, '/planning/obstacles', 10)
        self.marker_pub   = self.create_publisher(MarkerArray, '/planning/obstacle_markers', 10)

        # ── Periodic re-publish timer ─────────────────────────────────────────
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self._publish_map)

        self.get_logger().info(
            f'Obstacle Publisher Node started. '
            f'merge_radius={self.get_parameter("merge_radius").value} m | '
            f'forget_timeout={self.get_parameter("forget_timeout").value} s | '
            f'publish_rate={rate} Hz'
        )

    def _clear_callback(self, msg: Bool):
        if msg.data:
            self.obstacle_map.clear()
            self.get_logger().info('Obstacle map cleared.')

    # ── Incoming detection callback ───────────────────────────────────────────

    def callback(self, msg: Detection3DArray):
        max_base_h   = self.get_parameter('max_obstacle_base_height').value
        min_conf     = self.get_parameter('min_confidence').value
        source_frame = self.get_parameter('source_frame').value
        merge_radius = self.get_parameter('merge_radius').value
        ema_alpha    = self.get_parameter('ema_alpha').value

        world_frame  = self.get_parameter('world_frame').value

        # Use native TF2 to perfectly resolve the 6-frame tree offsets
        try:
            tf_stamped = self.tf_buffer.lookup_transform(
                world_frame,   # target frame
                source_frame,  # source frame (where the detections are)
                rclpy.time.Time(), # get the latest available
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(
                f'Waiting for TF tree to link {source_frame} to {world_frame}...',
                throttle_duration_sec=5.0,
            )
            return

        now = time.time()
        self.latest_stamp = msg.header.stamp

        for det in msg.detections:
            # Confidence filter (min_conf=0.0 by default — accept all LiDAR clusters)
            score    = det.results[0].hypothesis.score    if det.results else 1.0
            class_id = det.results[0].hypothesis.class_id if det.results else 'obstacle'
            if score < min_conf:
                continue

            # Height filter — evaluated in camera frame (Z is UP)
            bottom_z_cam = det.bbox.center.position.z - det.bbox.size.z / 2.0
            if bottom_z_cam > max_base_h:
                continue

            # Distance filter — 3D distance from camera origin in camera frame
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            cz = det.bbox.center.position.z
            dist_cam = math.sqrt(cx*cx + cy*cy + cz*cz)
            if dist_cam > self.get_parameter('max_detection_distance').value:
                continue

            # Transform obstacle centre to world frame
            pt_cam        = PointStamped()
            pt_cam.header = msg.header
            pt_cam.point  = det.bbox.center.position
            try:
                pt_world = tf2_geometry_msgs.do_transform_point(pt_cam, tf_stamped)
            except Exception as e:
                self.get_logger().warn(f'Transform failed: {e}')
                continue

            # Build world-frame detection
            out_det = Detection3D()
            out_det.header.frame_id       = world_frame
            out_det.header.stamp          = msg.header.stamp
            out_det.bbox.center.position.x = pt_world.point.x
            out_det.bbox.center.position.y = pt_world.point.y
            out_det.bbox.center.position.z = pt_world.point.z
            out_det.bbox.center.orientation.w = 1.0
            out_det.bbox.size.x = det.bbox.size.x
            out_det.bbox.size.y = det.bbox.size.y
            out_det.bbox.size.z = det.bbox.size.z
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = class_id
            hyp.hypothesis.score    = score
            out_det.results.append(hyp)

            # Merge into obstacle map: if a known obstacle is within merge_radius, update it;
            # otherwise add a new entry
            merged = False
            for entry in self.obstacle_map.values():
                existing = entry['detection']
                dx = existing.bbox.center.position.x - pt_world.point.x
                dy = existing.bbox.center.position.y - pt_world.point.y
                dz = existing.bbox.center.position.z - pt_world.point.z
                if math.sqrt(dx*dx + dy*dy + dz*dz) < merge_radius:
                    # Update in place with EMA (Exponential Moving Average) to eliminate jitter
                    existing.bbox.center.position.x = (1 - ema_alpha) * existing.bbox.center.position.x + ema_alpha * out_det.bbox.center.position.x
                    existing.bbox.center.position.y = (1 - ema_alpha) * existing.bbox.center.position.y + ema_alpha * out_det.bbox.center.position.y
                    existing.bbox.center.position.z = (1 - ema_alpha) * existing.bbox.center.position.z + ema_alpha * out_det.bbox.center.position.z
                    
                    existing.bbox.size.x = (1 - ema_alpha) * existing.bbox.size.x + ema_alpha * out_det.bbox.size.x
                    existing.bbox.size.y = (1 - ema_alpha) * existing.bbox.size.y + ema_alpha * out_det.bbox.size.y
                    existing.bbox.size.z = (1 - ema_alpha) * existing.bbox.size.z + ema_alpha * out_det.bbox.size.z
                    
                    # Keep highest confidence score
                    if out_det.results and existing.results:
                        existing.results[0].hypothesis.score = max(existing.results[0].hypothesis.score, out_det.results[0].hypothesis.score)

                    entry['last_seen']  = now
                    merged = True
                    break

            if not merged:
                if len(self.obstacle_map) >= self.get_parameter('max_obstacles').value:
                    # Map is full — drop the lowest-confidence entry to make room
                    oldest_key = min(
                        self.obstacle_map,
                        key=lambda k: self.obstacle_map[k]['detection'].results[0].hypothesis.score
                        if self.obstacle_map[k]['detection'].results else 0.0
                    )
                    del self.obstacle_map[oldest_key]

                self.obstacle_map[self._next_id] = {
                    'detection':  out_det,
                    'last_seen':  now,
                    'color_idx':  self._next_id,  # fixed forever — colour won't change
                }
                self._next_id += 1

        # Forget stale obstacles
        self._forget_old(now)

    # ── Periodic publisher ────────────────────────────────────────────────────

    def _publish_map(self):
        """Called by timer — publishes the full accumulated obstacle array."""
        if not self.obstacle_map:
            return

        world_frame = next(iter(self.obstacle_map.values()))['detection'].header.frame_id

        out = Detection3DArray()
        out.header.stamp    = self.latest_stamp if self.latest_stamp else self.get_clock().now().to_msg()
        out.header.frame_id = world_frame

        for entry in self.obstacle_map.values():
            out.detections.append(entry['detection'])

        self.obstacle_pub.publish(out)
        # Pass the full map (with color info) to the marker builder
        self._publish_markers(self.obstacle_map)

        self.get_logger().debug(f'Map published: {len(out.detections)} obstacle(s) in "{world_frame}"')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _forget_old(self, now: float):
        """Remove obstacles that have not been seen within forget_timeout seconds."""
        timeout = self.get_parameter('forget_timeout').value
        stale   = [k for k, v in self.obstacle_map.items() if now - v['last_seen'] > timeout]
        for k in stale:
            del self.obstacle_map[k]
        if stale:
            self.get_logger().debug(f'Forgot {len(stale)} stale obstacle(s).')

    def _publish_markers(self, obstacle_map: dict):
        """Build and publish world-frame MarkerArray for RViz."""
        if not obstacle_map:
            return

        world_frame = next(iter(obstacle_map.values()))['detection'].header.frame_id
        stamp       = self.latest_stamp if self.latest_stamp else self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        COLOURS = [
            ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.4),
            ColorRGBA(r=0.2, g=0.8, b=1.0, a=0.4),
            ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.4),
            ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.4),
            ColorRGBA(r=0.8, g=0.2, b=1.0, a=0.4),
            ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.4),
        ]

        for map_key, entry in obstacle_map.items():
            det      = entry['detection']
            # Use the obstacle's fixed color_idx so colour never changes after detection
            colour   = COLOURS[entry['color_idx'] % len(COLOURS)]
            class_id = det.results[0].hypothesis.class_id if det.results else 'object'
            score    = det.results[0].hypothesis.score    if det.results else 0.0
            pos      = det.bbox.center.position
            idx      = map_key  # stable marker ID — tied to the obstacle's map key

            # Semi-transparent filled cube
            cube = Marker()
            cube.header.frame_id    = world_frame
            cube.header.stamp       = stamp
            cube.ns                 = 'world_obstacle_box'
            cube.id                 = idx * 3
            cube.type               = Marker.CUBE
            cube.action             = Marker.ADD
            cube.pose.position      = pos
            cube.pose.orientation.w = 1.0
            cube.scale.x = max(det.bbox.size.x, 0.05)
            cube.scale.y = max(det.bbox.size.y, 0.05)
            cube.scale.z = max(det.bbox.size.z, 0.05)
            cube.color   = colour
            marker_array.markers.append(cube)

            # Wireframe outline
            wire = Marker()
            wire.header.frame_id    = world_frame
            wire.header.stamp       = stamp
            wire.ns                 = 'world_obstacle_wire'
            wire.id                 = idx * 3 + 1
            wire.type               = Marker.LINE_LIST
            wire.action             = Marker.ADD
            wire.pose.position      = pos
            wire.pose.orientation.w = 1.0
            wire.scale.x            = 0.04
            wire.color              = ColorRGBA(r=colour.r, g=colour.g, b=colour.b, a=1.0)

            hx, hy, hz = det.bbox.size.x/2, det.bbox.size.y/2, det.bbox.size.z/2
            corners = [
                Point(x= hx, y= hy, z= hz), Point(x=-hx, y= hy, z= hz),
                Point(x= hx, y=-hy, z= hz), Point(x=-hx, y=-hy, z= hz),
                Point(x= hx, y= hy, z=-hz), Point(x=-hx, y= hy, z=-hz),
                Point(x= hx, y=-hy, z=-hz), Point(x=-hx, y=-hy, z=-hz),
            ]
            for a, b in [(0,1),(2,3),(4,5),(6,7),(0,2),(1,3),(4,6),(5,7),(0,4),(1,5),(2,6),(3,7)]:
                wire.points.append(corners[a])
                wire.points.append(corners[b])
            marker_array.markers.append(wire)

            # Text label — class, confidence, world XY, size
            label = Marker()
            label.header.frame_id    = world_frame
            label.header.stamp       = stamp
            label.ns                 = 'world_obstacle_label'
            label.id                 = idx * 3 + 2
            label.type               = Marker.TEXT_VIEW_FACING
            label.action             = Marker.ADD
            label.pose.position.x    = pos.x
            label.pose.position.y    = pos.y
            label.pose.position.z    = pos.z + det.bbox.size.z / 2.0 + 0.15
            label.pose.orientation.w = 1.0
            label.scale.z            = 0.20
            label.color              = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = (
                f'{class_id} {score:.2f}\n'
                f'({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) m\n'
                f'{det.bbox.size.x:.1f}×{det.bbox.size.y:.1f}×{det.bbox.size.z:.1f} m'
            )
            marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
