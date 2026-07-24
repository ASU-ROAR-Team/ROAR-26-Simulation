#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np
import cv2  
from rclpy.qos import qos_profile_sensor_data 
import message_filters

import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation as R

class ZedDegradationNode(Node):
    def __init__(self):
        super().__init__('zed_degradation_node')
        
        self.bridge = CvBridge()

        # TF2 Setup for dynamic sun tracking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera Intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        
        self.X_factor = None
        self.Y_factor = None
        
        self.declare_parameter('washout_max', 0.85) # Maximum washout intensity when sun is centered
        
        self.gt_sun = None
        self.gt_rover = None
        
        self.gt_sub = self.create_subscription(
            TFMessage,
            '/world/rover_world/pose/info',
            self.gt_callback,
            10
        )

        # 1. Subscribe to camera_info to get intrinsic matrix K
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/zed2i/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # Synchronize depth and rgb
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed2i/depth', qos_profile=qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed2i/image_raw', qos_profile=qos_profile_sensor_data)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.depth_sub, self.rgb_sub], 10, 0.1)
        self.ts.registerCallback(self.sync_callback)

        # 4. Publishers
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', qos_profile_sensor_data)
        self.depth_vis_pub = self.create_publisher(Image, '/zed2i/depth_color_vis', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', qos_profile_sensor_data)
        self.pc_pub = self.create_publisher(PointCloud2, '/zed2i/points_updated', qos_profile_sensor_data)
        
        self.get_logger().info('ZED 2i Degradation Node Started with Dynamic TF2 Sun Glare (Depth + RGB) and PointCloud2.')

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            
            U, V = np.meshgrid(np.arange(msg.width), np.arange(msg.height))
            self.X_factor = (U - self.cx) / self.fx
            self.Y_factor = (V - self.cy) / self.fy

    def gt_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'sun_marker':
                self.gt_sun = transform.transform
            elif transform.child_frame_id == 'roar_rover':
                self.gt_rover = transform.transform

    def get_sun_pixel(self, header):
        if self.fx is None or self.gt_sun is None or self.gt_rover is None:
            self.get_logger().info(f"DEBUG None check - fx: {self.fx is None}, gt_sun: {self.gt_sun is None}, gt_rover: {self.gt_rover is None}")
            return None, None
            
        try:
            # 1. Math Ground Truth: World -> Rover
            t_WR = np.array([self.gt_rover.translation.x, self.gt_rover.translation.y, self.gt_rover.translation.z])
            q_WR = [self.gt_rover.rotation.x, self.gt_rover.rotation.y, self.gt_rover.rotation.z, self.gt_rover.rotation.w]
            rot_WR = R.from_quat(q_WR)
            
            # 2. Math Ground Truth: World -> Sun
            p_WS = np.array([self.gt_sun.translation.x, self.gt_sun.translation.y, self.gt_sun.translation.z])
            
            # 3. Calculate Sun relative to Rover (base_footprint physical)
            p_RS = rot_WR.inv().apply(p_WS - t_WR)
            
            sun_in_base = PointStamped()
            sun_in_base.header.frame_id = 'base_footprint'
            sun_in_base.point.x = p_RS[0]
            sun_in_base.point.y = p_RS[1]
            sun_in_base.point.z = p_RS[2]
            
            # 4. Use static TF tree to map from base_footprint to camera lens
            transform = self.tf_buffer.lookup_transform(
                'zed2i_depth_optical_frame', 
                'base_footprint',
                rclpy.time.Time() 
            )
            
            from tf2_geometry_msgs import do_transform_point
            sun_point_cam = do_transform_point(sun_in_base, transform)
            
            # UNCONDITIONAL DEBUG
            print(f"DEBUG: Sun in cam frame: x={sun_point_cam.point.x:.2f}, y={sun_point_cam.point.y:.2f}, z={sun_point_cam.point.z:.2f}")

            if sun_point_cam.point.z > 0.0:
                u = (self.fx * (sun_point_cam.point.x / sun_point_cam.point.z)) + self.cx
                v = (self.fy * (sun_point_cam.point.y / sun_point_cam.point.z)) + self.cy
                # DEBUG
                self.get_logger().info(f"Sun is at u={u}, v={v}, z={sun_point_cam.point.z}")
                return u, v
            else:
                self.get_logger().info(f"Sun is BEHIND camera! z={sun_point_cam.point.z}")
        except Exception as tf_ex:
            # DEBUG
            self.get_logger().info(f"TF Exception: {tf_ex}")
            pass
            
        return None, None

    def sync_callback(self, depth_msg: Image, rgb_msg: Image):
        if self.fx is None:
            return

        try:
            # ---------------------------------------------------------
            # PROCESS RGB
            # ---------------------------------------------------------
            cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8').copy()
            h, w, _ = cv_rgb.shape
            
            dust_probs = np.random.uniform(0.0, 1.0, size=(h, w))
            dust_mask = dust_probs < 0.015 
            dust_color = np.array([80, 120, 180], dtype=np.uint8) 
            cv_rgb[dust_mask] = dust_color

            u, v = self.get_sun_pixel(rgb_msg.header)
            
            if u is not None and v is not None:
                glare_radius = 150       
                if -glare_radius < u < w + glare_radius and -glare_radius < v < h + glare_radius:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u)**2 + (Y - v)**2)
                    
                    intensity = np.clip(1.0 - (dist_from_sun / glare_radius), 0, 1)
                    intensity = intensity ** 3
                    
                    bloom = np.zeros_like(cv_rgb, dtype=np.float32)
                    bloom[:,:,0] = 255.0 * intensity
                    bloom[:,:,1] = 255.0 * intensity
                    bloom[:,:,2] = 255.0 * intensity
                    
                    cv_rgb_float = cv_rgb.astype(np.float32)
                    cv_rgb_float += bloom
                    
                    dist_to_center = np.sqrt((u - (w/2))**2 + (v - (h/2))**2)
                    max_dist = np.sqrt((w/2)**2 + (h/2)**2)
                    
                    washout_max = self.get_parameter('washout_max').value
                    global_washout = np.clip(1.0 - (dist_to_center / (max_dist * 1.5)), 0.0, 1.0) * washout_max
                    
                    cv_rgb_float = (cv_rgb_float * (1.0 - global_washout)) + (255.0 * global_washout)
                    cv_rgb = np.clip(cv_rgb_float, 0, 255).astype(np.uint8)

            rgb_msg.header.frame_id = 'zed2i_depth_optical_frame'
            degraded_rgb_msg = self.bridge.cv2_to_imgmsg(cv_rgb, encoding='bgr8')
            degraded_rgb_msg.header = rgb_msg.header
            self.rgb_pub.publish(degraded_rgb_msg)

            # ---------------------------------------------------------
            # PROCESS DEPTH
            # ---------------------------------------------------------
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1').copy()
            valid_mask = np.isfinite(cv_depth) & (cv_depth > 0.2)
            
            z_values = cv_depth[valid_mask]
            stddev = 0.003 * (z_values ** 2)
            noise = np.random.normal(loc=0.0, scale=stddev)
            cv_depth[valid_mask] += noise

            dust_mask_depth = valid_mask & (dust_probs < 0.015)
            cv_depth[dust_mask_depth] = 0.4 

            if u is not None and v is not None:
                glare_radius_depth = 80       
                if -glare_radius_depth < u < w + glare_radius_depth and -glare_radius_depth < v < h + glare_radius_depth:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u)**2 + (Y - v)**2)
                    glare_mask = dist_from_sun < glare_radius_depth
                    cv_depth[glare_mask] = np.nan 

            depth_msg.header.frame_id = 'zed2i_depth_optical_frame'
            degraded_depth_msg = self.bridge.cv2_to_imgmsg(cv_depth, encoding='32FC1')
            degraded_depth_msg.header = depth_msg.header
            self.depth_pub.publish(degraded_depth_msg)

            vis_img = np.copy(cv_depth)
            vis_img[~np.isfinite(vis_img)] = 15.0
            vis_img = np.clip(vis_img, 0.0, 15.0)
            vis_norm = cv2.normalize(vis_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colorized_depth = cv2.applyColorMap(vis_norm, cv2.COLORMAP_JET)
            vis_msg = self.bridge.cv2_to_imgmsg(colorized_depth, encoding='bgr8')
            vis_msg.header = depth_msg.header
            self.depth_vis_pub.publish(vis_msg)
            
            # ---------------------------------------------------------
            # GENERATE POINTCLOUD2
            # ---------------------------------------------------------
            X = self.X_factor * cv_depth
            Y = self.Y_factor * cv_depth
            
            R_chan = cv_rgb[..., 2].astype(np.uint32)
            G_chan = cv_rgb[..., 1].astype(np.uint32)
            B_chan = cv_rgb[..., 0].astype(np.uint32)
            rgb_packed = (R_chan << 16) | (G_chan << 8) | B_chan
            rgb_packed = rgb_packed.view(np.float32)

            pc_valid = np.isfinite(cv_depth) & (cv_depth > 0.0)
            
            X_valid = X[pc_valid]
            Y_valid = Y[pc_valid]
            Z_valid = cv_depth[pc_valid]
            rgb_valid = rgb_packed[pc_valid]
            
            points = np.stack((X_valid, Y_valid, Z_valid, rgb_valid), axis=-1).astype(np.float32)
            
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
            self.get_logger().error(f'Failed to process images: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ZedDegradationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()