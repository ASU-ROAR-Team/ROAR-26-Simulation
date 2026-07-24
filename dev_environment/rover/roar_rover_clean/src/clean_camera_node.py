#!/usr/bin/env python3
"""
clean_camera_node.py

A simple pass-through node for clean simulation testing.
Subscribes to: /zed2i/depth and /zed2i/image_raw (raw Gazebo output)
Publishes to:  /zed2i/depth_updated, /zed2i/image_raw_updated, and /zed2i/points_updated (unified topics for SLAM/Perception)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from rclpy.qos import qos_profile_sensor_data
import message_filters
from cv_bridge import CvBridge
import numpy as np

class CleanCameraNode(Node):
    def __init__(self):
        super().__init__('clean_camera_node')

        self.bridge = CvBridge()
        
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.X_factor = None
        self.Y_factor = None

        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/zed2i/camera_info', self.camera_info_callback, qos_profile_sensor_data
        )

        # Synchronize depth and rgb
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed2i/depth', qos_profile=qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed2i/image_raw', qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)
        
        # Publishers
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', qos_profile_sensor_data)
        self.pc_pub = self.create_publisher(PointCloud2, '/zed2i/points_updated', qos_profile_sensor_data)

        self.get_logger().info('Clean Camera Node Started! Routing to _updated topics and publishing PointCloud2.')

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            
            U, V = np.meshgrid(np.arange(msg.width), np.arange(msg.height))
            self.X_factor = (U - self.cx) / self.fx
            self.Y_factor = (V - self.cy) / self.fy

    def sync_callback(self, depth_msg: Image, rgb_msg: Image):
        # Fix the frame_id before passing through or generating PointCloud
        depth_msg.header.frame_id = 'zed2i_depth_optical_frame'
        rgb_msg.header.frame_id = 'zed2i_depth_optical_frame'
        
        # Pass through the images
        self.depth_pub.publish(depth_msg)
        self.rgb_pub.publish(rgb_msg)
        
        # Generate and publish PointCloud2
        if self.fx is not None:
            try:
                Z = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
                RGB = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
                
                X = self.X_factor * Z
                Y = self.Y_factor * Z
                
                R = RGB[..., 2].astype(np.uint32)
                G = RGB[..., 1].astype(np.uint32)
                B = RGB[..., 0].astype(np.uint32)
                rgb_packed = (R << 16) | (G << 8) | B
                rgb_packed = rgb_packed.view(np.float32)

                valid = np.isfinite(Z) & (Z > 0.0)
                
                X = X[valid]
                Y = Y[valid]
                Z_valid = Z[valid]
                rgb_valid = rgb_packed[valid]
                
                points = np.stack((X, Y, Z_valid, rgb_valid), axis=-1).astype(np.float32)
                
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                
                pc2 = PointCloud2()
                pc2.header = depth_msg.header
                pc2.height = 1
                pc2.width = len(points)
                pc2.is_dense = False
                pc2.is_bigendian = False
                pc2.fields = fields
                pc2.point_step = 16
                pc2.row_step = 16 * len(points)
                pc2.data = points.tobytes()
                
                self.pc_pub.publish(pc2)
                
            except Exception as e:
                self.get_logger().error(f'Failed to generate point cloud: {str(e)}')

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