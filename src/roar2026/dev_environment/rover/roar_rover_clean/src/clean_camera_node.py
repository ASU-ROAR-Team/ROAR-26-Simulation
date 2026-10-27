#!/usr/bin/env python3
"""
clean_camera_node.py

Pass-through camera node for clean simulation testing.
Subscribes to: /zed2i/depth and /zed2i/image_raw (raw Gazebo output)
Publishes to:  /zed2i/depth_updated, /zed2i/image_raw_updated, and /zed2i/points_updated
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np


class CleanCameraNode(Node):
    def __init__(self):
        super().__init__('clean_camera_node')

        self.bridge = CvBridge()

        # Pre-initialize camera intrinsics from URDF (640x360, hfov=1.91986 rad)
        self.camera_info_received = False
        default_w, default_h, default_hfov = 640, 360, 1.91986
        default_fx = (default_w / 2.0) / math.tan(default_hfov / 2.0)
        self._set_intrinsics(default_w, default_h, default_fx, default_fx, default_w / 2.0, default_h / 2.0)

        self.latest_rgb = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/zed2i/camera_info', self.camera_info_callback, qos_profile_sensor_data
        )
        self.rgb_sub = self.create_subscription(
            Image, '/zed2i/image_raw', self.rgb_callback, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, '/zed2i/depth', self.depth_callback, qos_profile_sensor_data
        )

        # RELIABLE publishers are compatible with BOTH Best-Effort and Reliable subscribers
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', pub_qos)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', pub_qos)
        self.pc_pub = self.create_publisher(PointCloud2, '/zed2i/points_updated', pub_qos)

        self.get_logger().info('Clean Camera Node started (decoupled RGB/Depth + PointCloud2).')

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

    def rgb_callback(self, rgb_msg: Image):
        rgb_msg.header.frame_id = 'zed2i_depth_optical_frame'
        self.rgb_pub.publish(rgb_msg)
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception:
            pass

    def depth_callback(self, depth_msg: Image):
        depth_msg.header.frame_id = 'zed2i_depth_optical_frame'
        self.depth_pub.publish(depth_msg)

        try:
            Z = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            h, w = Z.shape
            if self.X_factor is None or self.X_factor.shape != (h, w):
                self._set_intrinsics(w, h, self.fx, self.fy, w / 2.0, h / 2.0)

            X = self.X_factor * Z
            Y = self.Y_factor * Z

            if self.latest_rgb is not None and self.latest_rgb.shape[:2] == (h, w):
                RGB = self.latest_rgb
                R = RGB[..., 2].astype(np.uint32)
                G = RGB[..., 1].astype(np.uint32)
                B = RGB[..., 0].astype(np.uint32)
                rgb_packed = ((R << 16) | (G << 8) | B).view(np.float32)
            else:
                neutral = np.uint32((200 << 16) | (200 << 8) | 200).view(np.float32)
                rgb_packed = np.full((h, w), neutral, dtype=np.float32)

            valid = np.isfinite(Z) & (Z > 0.0)
            points = np.stack((X[valid], Y[valid], Z[valid], rgb_packed[valid]), axis=-1).astype(np.float32)

            pc2 = PointCloud2()
            pc2.header = depth_msg.header
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
            self.get_logger().error(f'Failed to generate point cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CleanCameraNode()
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
