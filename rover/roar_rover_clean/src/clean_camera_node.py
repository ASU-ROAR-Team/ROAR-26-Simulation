#!/usr/bin/env python3
"""
clean_camera_node.py

A simple pass-through node for clean simulation testing.
Subscribes to: /zed2i/depth and /zed2i/image_raw (raw Gazebo output)
Publishes to:  /zed2i/depth_updated and /zed2i/image_raw_updated (unified topics for SLAM/Perception)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data

class CleanCameraNode(Node):
    def __init__(self):
        super().__init__('clean_camera_node')

        # Subscribers (Raw Gazebo Data)
        self.depth_sub = self.create_subscription(Image, '/zed2i/depth', self.depth_cb, qos_profile_sensor_data)
        self.rgb_sub = self.create_subscription(Image, '/zed2i/image_raw', self.rgb_cb, qos_profile_sensor_data)
        
        # Publishers (Unified Interface Data)
        self.depth_pub = self.create_publisher(Image, '/zed2i/depth_updated', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/zed2i/image_raw_updated', qos_profile_sensor_data)

        self.get_logger().info('Clean Camera Pass-Through Node Started! Routing to _updated topics.')

    def depth_cb(self, msg: Image):
        self.depth_pub.publish(msg)

    def rgb_cb(self, msg: Image):
        self.rgb_pub.publish(msg)

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