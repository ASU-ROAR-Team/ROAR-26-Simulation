#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2  
from rclpy.qos import qos_profile_sensor_data 

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

        # 2. Subscribe to the raw depth image
        self.depth_sub = self.create_subscription(
            Image,
            '/zed2i/depth',
            self.depth_callback,
            qos_profile_sensor_data
        )

        # 3. Subscribe to the raw RGB image
        self.rgb_sub = self.create_subscription(
            Image,
            '/zed2i/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data
        )

        # 4. Publishers
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', qos_profile_sensor_data)
        self.depth_vis_pub = self.create_publisher(Image, '/zed2i/depth_color_vis', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', qos_profile_sensor_data)
        
        self.get_logger().info('ZED 2i Degradation Node Started with Dynamic TF2 Sun Glare (Depth + RGB).')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.cx = msg.k[2]
        self.fy = msg.k[4]
        self.cy = msg.k[5]

    def gt_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'sun_marker':
                self.gt_sun = transform.transform
            elif transform.child_frame_id == 'roar_rover':
                self.gt_rover = transform.transform

    def get_sun_pixel(self, header):
        if self.fx is None or self.gt_sun is None or self.gt_rover is None:
            return None, None
            
        try:
            # 1. Math Ground Truth: World -> Rover
            t_WR = np.array([self.gt_rover.translation.x, self.gt_rover.translation.y, self.gt_rover.translation.z])
            q_WR = [self.gt_rover.rotation.x, self.gt_rover.rotation.y, self.gt_rover.rotation.z, self.gt_rover.rotation.w]
            rot_WR = R.from_quat(q_WR)
            
            # 2. Math Ground Truth: World -> Sun
            p_WS = np.array([self.gt_sun.translation.x, self.gt_sun.translation.y, self.gt_sun.translation.z])
            
            # 3. Calculate Sun relative to Rover (base_link physical)
            # P_W = rot_WR * P_R + t_WR  =>  P_R = rot_WR.inv() * (P_W - t_WR)
            p_RS = rot_WR.inv().apply(p_WS - t_WR)
            
            sun_in_base = PointStamped()
            sun_in_base.header.frame_id = 'base_link'
            sun_in_base.point.x = p_RS[0]
            sun_in_base.point.y = p_RS[1]
            sun_in_base.point.z = p_RS[2]
            
            # 4. Use static TF tree (no odometry drift) to map from base_link to camera lens
            transform = self.tf_buffer.lookup_transform(
                'zed2i_depth_optical_frame', 
                'base_link',
                rclpy.time.Time() 
            )
            
            from tf2_geometry_msgs import do_transform_point
            sun_point_cam = do_transform_point(sun_in_base, transform)
            
            # Check if the sun is in FRONT of the camera (Z > 0)
            if sun_point_cam.point.z > 0.0:
                # Project the 3D coordinate onto the 2D image plane using camera matrix
                u = (self.fx * (sun_point_cam.point.x / sun_point_cam.point.z)) + self.cx
                v = (self.fy * (sun_point_cam.point.y / sun_point_cam.point.z)) + self.cy
                return u, v
        except Exception as tf_ex:
            self.get_logger().error(f"TF or Math Error: {tf_ex}")
            pass
            
        return None, None

    def rgb_callback(self, msg):
        if self.fx is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8').copy()
            h, w, _ = cv_image.shape
            
            # ---------------------------------------------------------
            # ERROR 1: Dust Particles (RGB)
            # ---------------------------------------------------------
            dust_probs = np.random.uniform(0.0, 1.0, size=(h, w))
            dust_mask = dust_probs < 0.015 # 1.5% dust coverage
            # Mars dust color in BGR (Brownish/Orange)
            dust_color = np.array([80, 120, 180], dtype=np.uint8) 
            cv_image[dust_mask] = dust_color

            # ---------------------------------------------------------
            # ERROR 2: Dynamic TF2 Sun Glare Bloom (RGB)
            # ---------------------------------------------------------
            u, v = self.get_sun_pixel(msg.header)
            if u is not None and v is not None:
                glare_radius = 350       
                
                # Only calculate glare if the sun is reasonably close to the frame
                if -glare_radius < u < w + glare_radius and -glare_radius < v < h + glare_radius:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u)**2 + (Y - v)**2)
                    
                    # Create a bloom effect (soft white circle)
                    # For distance < glare_radius, calculate an intensity from 1.0 to 0.0
                    intensity = np.clip(1.0 - (dist_from_sun / glare_radius), 0, 1)
                    # Square the intensity for a hot core and soft falloff
                    intensity = intensity ** 3
                    
                    # Add white bloom to the image
                    bloom = np.zeros_like(cv_image, dtype=np.float32)
                    bloom[:,:,0] = 255.0 * intensity
                    bloom[:,:,1] = 255.0 * intensity
                    bloom[:,:,2] = 255.0 * intensity
                    
                    cv_image_float = cv_image.astype(np.float32)
                    cv_image_float += bloom
                    
                    # Also apply a global "Veiling Glare" (washout) if the sun is in the lens
                    # Calculate how close the sun is to the optical center
                    dist_to_center = np.sqrt((u - (w/2))**2 + (v - (h/2))**2)
                    max_dist = np.sqrt((w/2)**2 + (h/2)**2)
                    
                    # Global washout intensity (stronger when sun is centered)
                    washout_max = self.get_parameter('washout_max').value
                    global_washout = np.clip(1.0 - (dist_to_center / (max_dist * 1.5)), 0.0, 1.0) * washout_max
                    
                    # Wash out the image by blending with pure white
                    cv_image_float = (cv_image_float * (1.0 - global_washout)) + (255.0 * global_washout)
                    
                    cv_image = np.clip(cv_image_float, 0, 255).astype(np.uint8)

            # Publish Corrupted RGB
            msg.header.frame_id = 'zed2i_depth_optical_frame'
            degraded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            degraded_msg.header = msg.header
            self.rgb_pub.publish(degraded_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process rgb image: {str(e)}')


    def depth_callback(self, msg):
        if self.fx is None:
            return

        try:
            # Convert ROS Image to OpenCV Matrix
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1').copy()
            
            # Mask valid depth measurements
            valid_mask = np.isfinite(cv_image) & (cv_image > 0.2)
            
            # ---------------------------------------------------------
            # ERROR 1: Quadratic Depth Error
            # ---------------------------------------------------------
            z_values = cv_image[valid_mask]
            stddev = 0.003 * (z_values ** 2)
            noise = np.random.normal(loc=0.0, scale=stddev)
            cv_image[valid_mask] += noise

            # ---------------------------------------------------------
            # ERROR 2: Dust Particles (Depth)
            # ---------------------------------------------------------
            dust_probs = np.random.uniform(0.0, 1.0, size=cv_image.shape)
            dust_mask = valid_mask & (dust_probs < 0.015)
            cv_image[dust_mask] = 0.4 

            # ---------------------------------------------------------
            # ERROR 3: Dynamic TF2 Sun Glare (Sensor Blinding)
            # ---------------------------------------------------------
            u, v = self.get_sun_pixel(msg.header)
            if u is not None and v is not None:
                glare_radius = 180       
                h, w = cv_image.shape
                
                if -glare_radius < u < w + glare_radius and -glare_radius < v < h + glare_radius:
                    Y, X = np.ogrid[:h, :w]
                    dist_from_sun = np.sqrt((X - u)**2 + (Y - v)**2)
                    
                    # Blind the depth sensor by setting pixels in the glare radius to NaN
                    glare_mask = dist_from_sun < glare_radius
                    cv_image[glare_mask] = np.nan 

            # ---------------------------------------------------------
            # Publish Corrupted Map
            # ---------------------------------------------------------
            msg.header.frame_id = 'zed2i_depth_optical_frame'
            degraded_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='32FC1')
            degraded_msg.header = msg.header
            self.depth_pub.publish(degraded_msg)

            # ---------------------------------------------------------
            # VISUALIZATION (HEATMAP)
            # ---------------------------------------------------------
            vis_img = np.copy(cv_image)
            vis_img[~np.isfinite(vis_img)] = 15.0
            vis_img = np.clip(vis_img, 0.0, 15.0)
            vis_norm = cv2.normalize(vis_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            colorized_depth = cv2.applyColorMap(vis_norm, cv2.COLORMAP_JET)
            
            vis_msg = self.bridge.cv2_to_imgmsg(colorized_depth, encoding='bgr8')
            vis_msg.header = msg.header
            self.depth_vis_pub.publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f'Failed to process depth image: {str(e)}')

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