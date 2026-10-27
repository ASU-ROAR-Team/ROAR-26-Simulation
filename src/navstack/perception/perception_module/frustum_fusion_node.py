import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import message_filters
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


class FrustumFusionNode(Node):
    def __init__(self):
        super().__init__('frustum_fusion_node')

        # Parameters
        # lidar_frame: the frame the point cloud is published in
        # camera_frame: the OPTICAL frame used for K-matrix projection (Z must be depth)
        self.declare_parameter('lidar_frame', 'zed_left_camera_frame')
        self.declare_parameter('camera_frame', 'zed_left_camera_optical_frame')
        self.declare_parameter('match_radius', 150.0)

        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.match_radius = self.get_parameter('match_radius').value

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Camera intrinsics
        self.K = None
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed/zed_node/rgb/camera_info',
            self.camera_info_callback,
            10
        )

        # Subscriptions (sync)
        self.lidar_sub = message_filters.Subscriber(self, Detection3DArray, '/detection/lidar_clusters')
        self.camera_sub = message_filters.Subscriber(self, Detection2DArray, '/detection/camera_2d')
        self.mask_sub = message_filters.Subscriber(self, Image, '/detection/camera_mask')
        
        self.bridge = CvBridge()

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.camera_sub, self.mask_sub],
            queue_size=20,
            slop=1.0  # Increased to 1.0s slop to handle slower segmentation model framerates
        )
        self.sync.registerCallback(self.fused_callback)

        # Publishers
        self.fused_pub = self.create_publisher(Detection3DArray, '/detection/fused_3d', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/fused_markers', 10)

        self.get_logger().info("Frustum Fusion Node started.")

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.get_logger().info("Camera intrinsics received.")

    def fused_callback(self, lidar_msg, camera_msg, mask_msg):
        self.match_radius = self.get_parameter('match_radius').value

        if self.K is None:
            self.get_logger().warn("No camera info received yet.")
            return

        # TF: transform point cloud frame -> optical frame
        # camera_frame MUST be the optical frame so K-matrix projection (Z=depth) is correct.
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,           # target: optical frame (Z=depth)
                lidar_msg.header.frame_id,   # source: the frame in the point cloud header
                lidar_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.3)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed ('{lidar_msg.header.frame_id}' -> '{self.camera_frame}'): {e}")
            return

        fused_detections = Detection3DArray()
        fused_detections.header = lidar_msg.header

        try:
            mask_img = self.bridge.imgmsg_to_cv2(mask_msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Could not convert mask image: {e}")
            return
            
        H, W = mask_img.shape

        best_cluster_for_bbox = {}

        for det_3d in lidar_msg.detections:
            # Transform 3D cluster centroid into the optical frame
            point_lidar = PointStamped()
            point_lidar.header = lidar_msg.header
            point_lidar.point = det_3d.bbox.center.position

            point_opt = tf2_geometry_msgs.do_transform_point(point_lidar, transform)

            # In optical frame: Z is depth (forward). Skip points behind the camera.
            if point_opt.point.z <= 0:
                continue

            # Project 3D point into 2D image using camera intrinsics K
            p_opt = np.array([point_opt.point.x, point_opt.point.y, point_opt.point.z], dtype=np.float32)
            p_img = self.K @ p_opt
            u = float(p_img[0] / p_img[2])  # pixel column
            v = float(p_img[1] / p_img[2])  # pixel row
            
            u_int = int(round(u))
            v_int = int(round(v))

            best_match = None

            # Check a small window (radius) around the projected centroid to account for calibration slop
            # Increased to 30 pixels so we don't miss small rock centroids!
            window = 30  
            u_min = max(0, u_int - window)
            u_max = min(W, u_int + window + 1)
            v_min = max(0, v_int - window)
            v_max = min(H, v_int + window + 1)
            
            if u_min < u_max and v_min < v_max:
                patch = mask_img[v_min:v_max, u_min:u_max]
                unique_vals = np.unique(patch)
                
                for val in unique_vals:
                    if val > 0:  # 0 is background
                        # Found a mask hit! Look up the corresponding 2D detection by ID
                        for det_2d in camera_msg.detections:
                            if det_2d.id == str(val):
                                best_match = det_2d
                                break
                        if best_match is not None:
                            break # We have successfully matched the cluster to an object outline

            # Only process if matched
            if best_match is not None and best_match.results:
                dist_to_camera = np.sqrt(point_opt.point.x**2 + point_opt.point.y**2 + point_opt.point.z**2)
                box_id = best_match.id
                best_h = best_match.results[0]
                
                # If multiple 3D clusters fall inside the same 2D bounding box, keep the closest!
                if box_id not in best_cluster_for_bbox or dist_to_camera < best_cluster_for_bbox[box_id][0]:
                    best_cluster_for_bbox[box_id] = (dist_to_camera, det_3d, best_h)

        # Now publish ONLY the closest 3D cluster for each YOLO detection
        for box_id, (dist, det_fused, best_h) in best_cluster_for_bbox.items():
            det_fused.results.clear()

            fused_hyp = ObjectHypothesisWithPose()
            fused_hyp.hypothesis.class_id = best_h.hypothesis.class_id
            fused_hyp.hypothesis.score = best_h.hypothesis.score

            det_fused.results.append(fused_hyp)
            fused_detections.detections.append(det_fused)

        self.fused_pub.publish(fused_detections)

        # Build labeled 3D marker array for RViz
        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for idx, det in enumerate(fused_detections.detections):
            class_id = det.results[0].hypothesis.class_id if det.results else 'object'
            score    = det.results[0].hypothesis.score    if det.results else 0.0
            pos      = det.bbox.center.position

            # --- Wireframe box ---
            wire = Marker()
            wire.header = fused_detections.header
            wire.ns = 'fused_box'
            wire.id = idx * 2
            wire.type = Marker.LINE_LIST
            wire.action = Marker.ADD
            wire.pose.position = pos
            wire.pose.orientation.w = 1.0
            wire.scale.x = 0.04
            wire.color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=1.0)  # bright green
            wire.lifetime.sec = 0
            wire.lifetime.nanosec = 0  # Let DELETEALL handle cleanup to prevent flickering

            hx = det.bbox.size.x / 2.0
            hy = det.bbox.size.y / 2.0
            hz = det.bbox.size.z / 2.0
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

            # --- Floating text label ---
            text = Marker()
            text.header = fused_detections.header
            text.ns = 'fused_label'
            text.id = idx * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos.x
            text.pose.position.y = pos.y
            text.pose.position.z = pos.z + det.bbox.size.z / 2.0 + 0.15  # hover above box
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25  # text height in metres
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # white
            text.text = f"{class_id} {score:.2f}"
            text.lifetime.sec = 0
            text.lifetime.nanosec = 0
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrustumFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()