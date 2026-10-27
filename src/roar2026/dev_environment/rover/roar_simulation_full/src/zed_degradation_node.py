#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from cv_bridge import CvBridge
import numpy as np
import cv2

try:
    from tf2_ros import Buffer, TransformListener
    _HAS_TF2 = True
except ImportError:
    _HAS_TF2 = False


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion (x, y, z, w) to a 3x3 active rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


class ZedDegradationNode(Node):
    def __init__(self):
        super().__init__('zed_degradation_node')

        self.bridge = CvBridge()

        # TF2 setup for static base_footprint -> zed2i_depth_optical_frame lookup
        if _HAS_TF2:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = None
            self.tf_listener = None

        # Parameters (defaults match basic_rover.launch.py spawn poses)
        self.declare_parameter('washout_max', 0.85)
        self.declare_parameter('sun_x', 20.0)
        self.declare_parameter('sun_y', 20.0)
        self.declare_parameter('sun_z', 15.0)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_z', 0.35)
        self.declare_parameter('spawn_yaw', 1.5707)

        sun_x = float(self.get_parameter('sun_x').value)
        sun_y = float(self.get_parameter('sun_y').value)
        sun_z = float(self.get_parameter('sun_z').value)
        spawn_x = float(self.get_parameter('spawn_x').value)
        spawn_y = float(self.get_parameter('spawn_y').value)
        spawn_z = float(self.get_parameter('spawn_z').value)
        spawn_yaw = float(self.get_parameter('spawn_yaw').value)

        # Initialize Sun & Rover poses immediately so static sun_marker never leaves gt_sun=None
        self.sun_pos = np.array([sun_x, sun_y, sun_z], dtype=np.float64)
        half_yaw = 0.5 * spawn_yaw
        self.rover_pos = np.array([spawn_x, spawn_y, spawn_z], dtype=np.float64)
        self.rover_quat = np.array([0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)], dtype=np.float64)

        # Pre-initialize camera intrinsics from URDF (640x360, hfov=1.91986 rad)
        # Updated automatically once /zed2i/camera_info arrives.
        self.camera_info_received = False
        default_w, default_h, default_hfov = 640, 360, 1.91986
        default_fx = (default_w / 2.0) / math.tan(default_hfov / 2.0)
        self._set_intrinsics(default_w, default_h, default_fx, default_fx, default_w / 2.0, default_h / 2.0)

        self.latest_rgb = None
        self.latest_dust_mask = None
        self._logged_first_frame = False

        # Subscriptions
        self.gt_sub = self.create_subscription(
            TFMessage,
            '/world/rover_world/pose/info',
            self.gt_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed2i/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        self.rgb_sub = self.create_subscription(
            Image,
            '/zed2i/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/zed2i/depth',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # RELIABLE publishers are compatible with BOTH Best-Effort (perception/sensor_data)
        # and Reliable (default RViz / CLI) subscribers.
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', pub_qos)
        self.depth_vis_pub = self.create_publisher(Image, '/zed2i/depth_color_vis', pub_qos)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', pub_qos)
        self.pc_pub = self.create_publisher(PointCloud2, '/zed2i/points_updated', pub_qos)

        self.get_logger().info(
            f'ZED 2i Degradation Node started (sun={self.sun_pos.tolist()}, '
            'decoupled RGB/Depth + dynamic Sun Glare + PointCloud2).'
        )

    def _set_intrinsics(self, width: int, height: int, fx: float, fy: float, cx: float, cy: float):
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        U, V = np.meshgrid(np.arange(width, dtype=np.float32), np.arange(height, dtype=np.float32))
        self.X_factor = (U - self.cx) / self.fx
        self.Y_factor = (V - self.cy) / self.fy

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self._set_intrinsics(msg.width, msg.height, msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics locked from /zed2i/camera_info: '
                f'{msg.width}x{msg.height}, fx={self.fx:.2f}, fy={self.fy:.2f}'
            )

    def gt_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            name = transform.child_frame_id
            t = transform.transform.translation
            q = transform.transform.rotation
            if name == 'sun_marker' or name.endswith('::sun_marker') or name.endswith('/sun_marker'):
                self.sun_pos = np.array([t.x, t.y, t.z], dtype=np.float64)
            elif name == 'roar_rover' or name.endswith('::roar_rover') or name.endswith('/roar_rover'):
                self.rover_pos = np.array([t.x, t.y, t.z], dtype=np.float64)
                self.rover_quat = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)

    def get_sun_pixel(self):
        if self.fx is None or self.sun_pos is None or self.rover_pos is None:
            return None, None

        try:
            # 1. World -> Rover (base_footprint)
            rot_wr = quat_to_rot_matrix(
                self.rover_quat[0], self.rover_quat[1],
                self.rover_quat[2], self.rover_quat[3]
            )
            p_rs = rot_wr.T @ (self.sun_pos - self.rover_pos)

            # 2. base_footprint -> zed2i_depth_optical_frame (live TF2 with URDF static fallback)
            p_cam = None
            if self.tf_buffer is not None:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        'zed2i_depth_optical_frame',
                        'base_footprint',
                        rclpy.time.Time()
                    )
                    tr = tf_msg.transform.translation
                    rq = tf_msg.transform.rotation
                    rot_cb = quat_to_rot_matrix(rq.x, rq.y, rq.z, rq.w)
                    p_cam = rot_cb @ p_rs + np.array([tr.x, tr.y, tr.z], dtype=np.float64)
                except Exception:
                    p_cam = None

            if p_cam is None:
                # Exact URDF static chain:
                # base_footprint -> zed2i_link: xyz=(0.35, 0.0, 0.25), rpy=(0, 0, 0)
                # zed2i_link -> zed2i_depth_optical_frame: rpy=(-pi/2, 0, -pi/2)
                p_link = p_rs - np.array([0.35, 0.0, 0.25], dtype=np.float64)
                p_cam = np.array([-p_link[1], -p_link[2], p_link[0]], dtype=np.float64)

            if p_cam[2] > 0.0:
                u = (self.fx * (p_cam[0] / p_cam[2])) + self.cx
                v = (self.fy * (p_cam[1] / p_cam[2])) + self.cy
                return float(u), float(v)
        except Exception as ex:
            self.get_logger().warn(f'Sun projection error: {ex}', throttle_duration_sec=5.0)

        return None, None

    def rgb_callback(self, rgb_msg: Image):
        try:
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8').copy()
            h, w, _ = cv_rgb.shape

            # Mars atmospheric dust speckle
            dust_probs = np.random.uniform(0.0, 1.0, size=(h, w))
            dust_mask = dust_probs < 0.015
            cv_rgb[dust_mask] = np.array([80, 120, 180], dtype=np.uint8)

            u, v = self.get_sun_pixel()

            if u is not None and v is not None:
                glare_radius = 150.0
                if -glare_radius < u < w + glare_radius and -glare_radius < v < h + glare_radius:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u) ** 2 + (Y - v) ** 2)

                    norm_dist = np.clip(1.0 - (dist_from_sun / glare_radius), 0.0, 1.0)
                    core = (norm_dist ** 2.5)[..., np.newaxis]
                    corona = (norm_dist ** 1.4)[..., np.newaxis]

                    # Bright white solar core + warm golden-white lens corona (BGR)
                    bloom = (
                        core * np.array([255.0, 255.0, 255.0], dtype=np.float32)
                        + corona * np.array([90.0, 190.0, 255.0], dtype=np.float32) * 0.65
                    )

                    cv_rgb_float = cv_rgb.astype(np.float32) + bloom

                    dist_to_center = math.sqrt((u - (w / 2.0)) ** 2 + (v - (h / 2.0)) ** 2)
                    max_dist = math.sqrt((w / 2.0) ** 2 + (h / 2.0) ** 2)

                    washout_max = float(self.get_parameter('washout_max').value)
                    global_washout = float(
                        np.clip(1.0 - (dist_to_center / (max_dist * 1.5)), 0.0, 1.0) * washout_max
                    )

                    cv_rgb_float = (cv_rgb_float * (1.0 - global_washout)) + (255.0 * global_washout)
                    cv_rgb = np.clip(cv_rgb_float, 0, 255).astype(np.uint8)

            self.latest_rgb = cv_rgb
            self.latest_dust_mask = dust_mask

            degraded_rgb_msg = self.bridge.cv2_to_imgmsg(cv_rgb, encoding='bgr8')
            degraded_rgb_msg.header.stamp = rgb_msg.header.stamp
            degraded_rgb_msg.header.frame_id = 'zed2i_depth_optical_frame'
            self.rgb_pub.publish(degraded_rgb_msg)

            if not self._logged_first_frame:
                self._logged_first_frame = True
                self.get_logger().info(
                    f'Publishing /zed2i/image_raw_updated ({w}x{h}), initial sun pixel=({u}, {v})'
                )
        except Exception as e:
            self.get_logger().error(f'Failed to process RGB image: {e}')

    def depth_callback(self, depth_msg: Image):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1').copy()
            h, w = cv_depth.shape

            if self.X_factor is None or self.X_factor.shape != (h, w):
                self._set_intrinsics(w, h, self.fx, self.fy, w / 2.0, h / 2.0)

            valid_mask = np.isfinite(cv_depth) & (cv_depth > 0.2)

            # Distance-quadratic stereo depth noise
            z_values = cv_depth[valid_mask]
            stddev = 0.003 * (z_values ** 2)
            cv_depth[valid_mask] += np.random.normal(loc=0.0, scale=stddev).astype(np.float32)

            # Mars dust depth dropouts
            if self.latest_dust_mask is not None and self.latest_dust_mask.shape == (h, w):
                dust_mask_depth = valid_mask & self.latest_dust_mask
            else:
                dust_mask_depth = valid_mask & (np.random.uniform(0.0, 1.0, size=(h, w)) < 0.015)
            cv_depth[dust_mask_depth] = 0.4

            # Sun glare stereo depth blindness
            u, v = self.get_sun_pixel()
            if u is not None and v is not None:
                glare_radius_depth = 80.0
                if -glare_radius_depth < u < w + glare_radius_depth and -glare_radius_depth < v < h + glare_radius_depth:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u) ** 2 + (Y - v) ** 2)
                    cv_depth[dist_from_sun < glare_radius_depth] = np.nan

            degraded_depth_msg = self.bridge.cv2_to_imgmsg(cv_depth, encoding='32FC1')
            degraded_depth_msg.header.stamp = depth_msg.header.stamp
            degraded_depth_msg.header.frame_id = 'zed2i_depth_optical_frame'
            self.depth_pub.publish(degraded_depth_msg)

            vis_img = np.copy(cv_depth)
            vis_img[~np.isfinite(vis_img)] = 15.0
            vis_img = np.clip(vis_img, 0.0, 15.0)
            vis_norm = cv2.normalize(vis_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colorized_depth = cv2.applyColorMap(vis_norm, cv2.COLORMAP_JET)
            vis_msg = self.bridge.cv2_to_imgmsg(colorized_depth, encoding='bgr8')
            vis_msg.header = degraded_depth_msg.header
            self.depth_vis_pub.publish(vis_msg)

            # Generate PointCloud2
            X = self.X_factor * cv_depth
            Y = self.Y_factor * cv_depth

            if self.latest_rgb is not None and self.latest_rgb.shape[:2] == (h, w):
                rgb_src = self.latest_rgb
                R_chan = rgb_src[..., 2].astype(np.uint32)
                G_chan = rgb_src[..., 1].astype(np.uint32)
                B_chan = rgb_src[..., 0].astype(np.uint32)
                rgb_packed = ((R_chan << 16) | (G_chan << 8) | B_chan).view(np.float32)
            else:
                neutral = np.uint32((200 << 16) | (200 << 8) | 200).view(np.float32)
                rgb_packed = np.full((h, w), neutral, dtype=np.float32)

            pc_valid = np.isfinite(cv_depth) & (cv_depth > 0.0)
            points = np.stack(
                (X[pc_valid], Y[pc_valid], cv_depth[pc_valid], rgb_packed[pc_valid]),
                axis=-1
            ).astype(np.float32)

            pc2 = PointCloud2()
            pc2.header = degraded_depth_msg.header
            pc2.height = 1
            pc2.width = len(points)
            pc2.is_dense = False
            pc2.is_bigendian = False
            pc2.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            pc2.point_step = 16
            pc2.row_step = 16 * len(points)
            pc2.data = points.tobytes()

            self.pc_pub.publish(pc2)
        except Exception as e:
            self.get_logger().error(f'Failed to process depth/pointcloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ZedDegradationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
