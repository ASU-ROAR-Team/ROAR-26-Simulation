import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3DArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import struct


class BEVVisualizerNode(Node):
    def __init__(self):
        super().__init__('bev_visualizer_node')

        # BEV Parameters
        self.declare_parameter('range_x', 10.0)
        self.declare_parameter('range_y', 10.0)
        self.declare_parameter('resolution', 0.05)

        # RGB option
        self.declare_parameter('use_rgb', False)

        self.range_x = self.get_parameter('range_x').value
        self.range_y = self.get_parameter('range_y').value
        self.resolution = self.get_parameter('resolution').value
        self.use_rgb = self.get_parameter('use_rgb').value

        self.width = int(2 * self.range_y / self.resolution)
        self.height = int(2 * self.range_x / self.resolution)

        self.bridge = CvBridge()

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/zed/zed_node/point_cloud/cloud_registered',
            self.lidar_callback,
            10
        )

        self.fused_sub = self.create_subscription(
            Detection3DArray,
            '/detection/fused_3d',
            self.fused_callback,
            10
        )

        self.latest_fused = None

        self.bev_pub = self.create_publisher(Image, '/visualization/bev_image', 10)

        self.get_logger().info(f"BEV Visualizer Node started. Size: {self.width}x{self.height}")

    def fused_callback(self, msg):
        self.latest_fused = msg

    def lidar_callback(self, msg):
        self.use_rgb = self.get_parameter('use_rgb').value

        if self.use_rgb:
            points, colors = self.pointcloud2_to_numpy_xyzrgb(msg)
        else:
            points = self.pointcloud2_to_numpy_xyz(msg)
            colors = None

        if points.shape[0] == 0:
            return

        # Filter by range (standard camera frame: X is forward, Y is left, Z is UP)
        mask = (
            (points[:, 1] > -self.range_y) & (points[:, 1] < self.range_y) &  # Y (left/right maps to range_y for BEV)
            (points[:, 0] > 0) & (points[:, 0] < self.range_x)                # X (forward maps to range_x for BEV)
        )
        points = points[mask]
        if colors is not None:
            colors = colors[mask]

        if points.shape[0] == 0:
            return

        # Convert to BEV pixel coordinates
        # Map Y (left) to x_pix (left-right on image). Left is positive Y, so map positive Y to left side of image (smaller x_pix)
        x_pix = ((self.range_y - points[:, 1]) / self.resolution).astype(np.int32)
        # Map X (forward) to y_pix (bottom-up on image). Forward is positive X, map to top of image (smaller y_pix)
        y_pix = ((self.range_x - points[:, 0]) / self.resolution).astype(np.int32)

        bev_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        valid_mask = (
            (x_pix >= 0) & (x_pix < self.width) &
            (y_pix >= 0) & (y_pix < self.height)
        )

        if colors is None:
            bev_img[y_pix[valid_mask], x_pix[valid_mask]] = (200, 200, 200)
        else:
            # Apply point colors
            bev_img[y_pix[valid_mask], x_pix[valid_mask]] = colors[valid_mask]

        # Draw fused detections
        if self.latest_fused is not None:
            for det in self.latest_fused.detections:
                # Standard camera frame: X forward, Y left
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                sx = det.bbox.size.x
                sy = det.bbox.size.y

                px = int((self.range_y - cy) / self.resolution)
                py = int((self.range_x - cx) / self.resolution)

                pw = int(sy / self.resolution)
                ph = int(sx / self.resolution)

                cv2.rectangle(
                    bev_img,
                    (px - pw // 2, py - ph // 2),
                    (px + pw // 2, py + ph // 2),
                    (0, 0, 255),
                    2
                )

                if det.results:
                    cls_id = det.results[0].hypothesis.class_id
                    cv2.putText(
                        bev_img,
                        f"{cls_id}",
                        (px - pw // 2, py - ph // 2 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 255, 0),
                        1
                    )

        bev_msg = self.bridge.cv2_to_imgmsg(bev_img, encoding='bgr8')
        bev_msg.header = msg.header
        self.bev_pub.publish(bev_msg)

    def pointcloud2_to_numpy_xyz(self, msg):
        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(gen), dtype=np.float32)

        if points.size == 0:
            return np.zeros((0, 3), dtype=np.float32)

        return points[:, :3]

    def pointcloud2_to_numpy_xyzrgb(self, msg):
        # Try rgb first, then rgba
        field_names = [f.name for f in msg.fields]

        if "rgb" in field_names:
            rgb_field = "rgb"
        elif "rgba" in field_names:
            rgb_field = "rgba"
        else:
            self.get_logger().warn("No rgb/rgba field found. Using gray points.")
            pts = self.pointcloud2_to_numpy_xyz(msg)
            return pts, None

        gen = point_cloud2.read_points(msg, field_names=("x", "y", "z", rgb_field), skip_nans=True)
        dtype = [('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.float32)]
        structured_array = np.fromiter(gen, dtype=dtype)

        if structured_array.size == 0:
            return np.zeros((0, 3), dtype=np.float32), None

        data = structured_array.view(np.float32).reshape(-1, 4)
        points = data[:, 0:3]
        rgb_floats = data[:, 3]

        colors = np.zeros((rgb_floats.shape[0], 3), dtype=np.uint8)

        for i, rgb in enumerate(rgb_floats):
            rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
            r = (rgb_int >> 16) & 255
            g = (rgb_int >> 8) & 255
            b = rgb_int & 255
            colors[i] = (b, g, r)  # OpenCV uses BGR

        return points, colors


def main(args=None):
    rclpy.init(args=args)
    node = BEVVisualizerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()