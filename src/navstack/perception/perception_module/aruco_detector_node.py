#!/usr/bin/env python3
"""
ArUco Detector Node
===================
Detects ArUco markers in the ZED left camera image and publishes:

  /detection/aruco_3d          – vision_msgs/Detection3DArray
                                 One Detection3D per marker.  The class_id is
                                 "aruco_<ID>" and the bbox centre carries the
                                 full 6-DOF pose (position + quaternion).

  /detection/aruco_poses        – geometry_msgs/PoseArray
                                 Convenience flat list of poses indexed by
                                 marker order (use alongside the array above
                                 when only XYZ+orientation is needed).

  /visualization/aruco_markers  – visualization_msgs/MarkerArray
                                 3-D boxes + axis frames + text labels for RViz.

  /detection/aruco_image        – sensor_msgs/Image
                                 Debug image with drawn corners and axes.

All tunable parameters (marker_size, camera topics …) live in
  config/perception_params.yaml
and are loaded at launch time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from active_mapping_msgs.msg import ArucoDetection, ArucoDetectionArray

from cv_bridge import CvBridge


# ────────────────────────────────────────────────────────────────────────────
# Pure-OpenCV ArUco tracker helper
# ────────────────────────────────────────────────────────────────────────────

class ArucoTracker:
    """
    Auto-detects markers from all major ArUco / AprilTag dictionaries.
    Uses SOLVEPNP_IPPE_SQUARE for accurate 6-DOF pose estimation.
    """

    _DICTS = [
        cv2.aruco.DICT_4X4_50,   cv2.aruco.DICT_4X4_100,
        cv2.aruco.DICT_4X4_250,  cv2.aruco.DICT_4X4_1000,
        cv2.aruco.DICT_5X5_50,   cv2.aruco.DICT_5X5_100,
        cv2.aruco.DICT_5X5_250,  cv2.aruco.DICT_5X5_1000,
        cv2.aruco.DICT_6X6_50,   cv2.aruco.DICT_6X6_100,
        cv2.aruco.DICT_6X6_250,  cv2.aruco.DICT_6X6_1000,
        cv2.aruco.DICT_7X7_50,   cv2.aruco.DICT_7X7_100,
        cv2.aruco.DICT_7X7_250,  cv2.aruco.DICT_7X7_1000,
        cv2.aruco.DICT_ARUCO_ORIGINAL,
        cv2.aruco.DICT_APRILTAG_16h5,  cv2.aruco.DICT_APRILTAG_25h9,
        cv2.aruco.DICT_APRILTAG_36h10, cv2.aruco.DICT_APRILTAG_36h11,
    ]

    def __init__(self):
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self._params = cv2.aruco.DetectorParameters_create()
        else:
            self._params = cv2.aruco.DetectorParameters()
        self._use_new_api = hasattr(cv2.aruco, 'ArucoDetector')

    def detect(self, image: np.ndarray):
        """Return (corners, ids, rejected) from the first dictionary that hits."""
        for d in self._DICTS:
            adict = cv2.aruco.getPredefinedDictionary(d)
            if self._use_new_api:
                det = cv2.aruco.ArucoDetector(adict, self._params)
                corners, ids, rejected = det.detectMarkers(image)
            else:
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    image, adict, parameters=self._params)
            if ids is not None and len(ids) > 0:
                return corners, ids, rejected
        return None, None, None

    def estimate_poses(self, corners, marker_size: float, K, dist):
        """
        Returns rvecs, tvecs as lists (one entry per detected marker).
        Uses SOLVEPNP_IPPE_SQUARE — the most accurate method for square targets.
        """
        if not corners:
            return [], []
        half = marker_size / 2.0
        obj_pts = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float32)

        rvecs, tvecs = [], []
        for corner in corners:
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts,
                corner.reshape(-1, 2).astype(np.float32),
                K, dist,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ok:
                rvecs.append(rvec)
                tvecs.append(tvec)
        return rvecs, tvecs


# ────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ────────────────────────────────────────────────────────────────────────────

class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')
        self.bridge = CvBridge()

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('camera_topic',      '/zed2i/image_raw_updated')   # simulation topic
        self.declare_parameter('camera_info_topic', '/zed2i/camera_info')           # simulation topic
        self.declare_parameter('aruco_dict_type',   0)    # unused in auto-detect mode
        self.declare_parameter('visualize',         True)
        self.declare_parameter('marker_size',       0.15) # [m] physical marker side

        self.camera_topic  = self.get_parameter('camera_topic').value
        self.info_topic    = self.get_parameter('camera_info_topic').value
        self.visualize     = self.get_parameter('visualize').value
        self.marker_size   = self.get_parameter('marker_size').value

        self.tracker = ArucoTracker()

        self.camera_matrix = None
        self.dist_coeffs   = None

        # Fallback ZED2i intrinsics (used until /camera_info arrives)
        self._fallback_K    = np.array([[528.0, 0.0, 640.0],
                                        [0.0, 528.0, 360.0],
                                        [0.0, 0.0, 1.0]])
        self._fallback_dist = np.zeros(5)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(Image,      self.camera_topic, self._image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic,   self._info_cb,  10)

        # ── Publishers ───────────────────────────────────────────────────
        # Full 3-D detections (ID + 6-DOF pose + bbox size)
        self.det3d_pub   = self.create_publisher(Detection3DArray, '/detection/aruco_3d',     10)
        # Flat PoseArray for convenience (same order as Detection3DArray)
        self.poses_pub   = self.create_publisher(PoseArray,         '/detection/aruco_poses',  10)
        # Custom message format for SLAM
        self.slam_pub    = self.create_publisher(ArucoDetectionArray, '/aruco/detections', 10)
        # RViz markers
        self.marker_pub  = self.create_publisher(MarkerArray,       '/visualization/aruco_markers', 10)
        # Debug image with axes drawn
        self.debug_pub   = self.create_publisher(Image,             '/detection/aruco_image',  10)

        self.get_logger().info('ArUco Detector Node started.')
        self.get_logger().info(f'Listening on: {self.camera_topic}')
        self.get_logger().info(f'Marker size:  {self.marker_size} m  |  auto-dict-detect ON')

    # ── Callbacks ────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received from /camera_info.')

    def _image_cb(self, image_msg: Image):
        self.marker_size = self.get_parameter('marker_size').value
        self.visualize   = self.get_parameter('visualize').value

        try:
            cv_img = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        except Exception as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        K    = self.camera_matrix if self.camera_matrix is not None else self._fallback_K
        dist = self.dist_coeffs   if self.dist_coeffs   is not None else self._fallback_dist

        corners, ids, _ = self.tracker.detect(cv_img)

        # ── Build output messages ─────────────────────────────────────
        det3d_array = Detection3DArray()
        det3d_array.header = image_msg.header

        pose_array = PoseArray()
        pose_array.header = image_msg.header

        slam_array = ArucoDetectionArray()
        slam_array.header = image_msg.header

        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if ids is not None:
            rvecs, tvecs = self.tracker.estimate_poses(corners, self.marker_size, K, dist)

            if self.visualize and rvecs:
                cv_img = cv_img.copy()
                cv2.aruco.drawDetectedMarkers(cv_img, corners, ids)
                for rv, tv in zip(rvecs, tvecs):
                    cv2.drawFrameAxes(cv_img, K, dist, rv, tv, self.marker_size * 0.5)

            for i, (marker_id_arr, rvec, tvec) in enumerate(zip(ids, rvecs, tvecs)):
                marker_id = int(marker_id_arr[0])
                rvec_flat = rvec.flatten()
                tvec_flat = tvec.flatten()

                # ── Convert rvec → rotation matrix → quaternion ───────
                rot_mat, _ = cv2.Rodrigues(rvec_flat)
                quat = R.from_matrix(rot_mat).as_quat()  # [x, y, z, w]

                # ── Detection3D ───────────────────────────────────────
                det3d = Detection3D()
                det3d.header = image_msg.header

                det3d.bbox.center.position.x    = float(tvec_flat[0])
                det3d.bbox.center.position.y    = float(tvec_flat[1])
                det3d.bbox.center.position.z    = float(tvec_flat[2])
                det3d.bbox.center.orientation.x = float(quat[0])
                det3d.bbox.center.orientation.y = float(quat[1])
                det3d.bbox.center.orientation.z = float(quat[2])
                det3d.bbox.center.orientation.w = float(quat[3])
                det3d.bbox.size.x = self.marker_size
                det3d.bbox.size.y = self.marker_size
                det3d.bbox.size.z = 0.02  # 2 cm "thickness" for the flat board

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f'aruco_{marker_id}'
                hyp.hypothesis.score    = 1.0
                det3d.results.append(hyp)
                det3d_array.detections.append(det3d)

                # ── PoseArray entry ───────────────────────────────────
                p = Pose()
                p.position.x    = float(tvec_flat[0])
                p.position.y    = float(tvec_flat[1])
                p.position.z    = float(tvec_flat[2])
                p.orientation.x = float(quat[0])
                p.orientation.y = float(quat[1])
                p.orientation.z = float(quat[2])
                p.orientation.w = float(quat[3])
                pose_array.poses.append(p)

                # ── SLAM Message entry ────────────────────────────────
                slam_det = ArucoDetection()
                slam_det.id = marker_id
                slam_det.pose_in_camera = p
                slam_det.distance = float(np.linalg.norm(tvec_flat))
                slam_array.detections.append(slam_det)

                # ── RViz: 3-D box ─────────────────────────────────────
                cube = Marker()
                cube.header          = image_msg.header
                cube.ns              = 'aruco_box'
                cube.id              = marker_id
                cube.type            = Marker.CUBE
                cube.action          = Marker.ADD
                cube.pose            = det3d.bbox.center
                cube.scale.x         = self.marker_size
                cube.scale.y         = self.marker_size
                cube.scale.z         = 0.02
                cube.color           = ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.7)  # magenta
                cube.lifetime.nanosec = 500_000_000
                marker_array.markers.append(cube)

                # ── RViz: coordinate axes (X=red, Y=green, Z=blue) ────
                axis_scale = self.marker_size * 0.6
                axis_colors = [
                    ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # X – red
                    ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # Y – green
                    ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # Z – blue
                ]
                # Direction vectors of the marker axes in camera frame
                axis_dirs = [rot_mat[:, 0], rot_mat[:, 1], rot_mat[:, 2]]
                origin = np.array(tvec_flat)

                for ax_i, (color, direction) in enumerate(zip(axis_colors, axis_dirs)):
                    tip = origin + direction * axis_scale
                    arrow = Marker()
                    arrow.header          = image_msg.header
                    arrow.ns              = f'aruco_axis_{ax_i}'
                    arrow.id              = marker_id * 10 + ax_i
                    arrow.type            = Marker.ARROW
                    arrow.action          = Marker.ADD
                    arrow.scale.x         = 0.01   # shaft diameter
                    arrow.scale.y         = 0.02   # head diameter
                    arrow.scale.z         = 0.04   # head length
                    arrow.color           = color
                    arrow.lifetime.nanosec = 500_000_000
                    arrow.points.append(Point(x=float(origin[0]),
                                              y=float(origin[1]),
                                              z=float(origin[2])))
                    arrow.points.append(Point(x=float(tip[0]),
                                              y=float(tip[1]),
                                              z=float(tip[2])))
                    marker_array.markers.append(arrow)

                # ── RViz: ID label ─────────────────────────────────────
                dist_m = float(np.linalg.norm(tvec_flat))
                text = Marker()
                text.header            = image_msg.header
                text.ns                = 'aruco_label'
                text.id                = marker_id + 10000
                text.type              = Marker.TEXT_VIEW_FACING
                text.action            = Marker.ADD
                text.pose.position.x   = float(tvec_flat[0])
                text.pose.position.y   = float(tvec_flat[1])
                text.pose.position.z   = float(tvec_flat[2]) + self.marker_size * 0.7
                text.pose.orientation.w = 1.0
                text.scale.z           = 0.10
                text.color             = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                text.text              = f'ArUco {marker_id}\n{dist_m:.2f} m'
                text.lifetime.nanosec  = 500_000_000
                marker_array.markers.append(text)

        # ── Publish all topics ────────────────────────────────────────
        self.det3d_pub.publish(det3d_array)
        self.poses_pub.publish(pose_array)
        self.slam_pub.publish(slam_array)
        self.marker_pub.publish(marker_array)

        if self.visualize:
            try:
                dbg_msg = self.bridge.cv2_to_imgmsg(cv_img, 'bgr8')
                dbg_msg.header = image_msg.header
                self.debug_pub.publish(dbg_msg)
            except Exception as exc:
                self.get_logger().error(f'Debug image publish error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
