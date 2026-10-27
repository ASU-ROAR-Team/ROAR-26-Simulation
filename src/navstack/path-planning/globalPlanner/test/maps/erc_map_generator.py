#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
import math
import numpy as np

class ErcMapGenerator(Node):
    def __init__(self):
        super().__init__('erc_map_generator')
        
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.resolution = self.get_parameter('resolution').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        
        self.rx = self.start_x
        self.ry = self.start_y
        self.wall_triggered = False
        
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Build base map once (to avoid generating noise on every tick)
        self.base_data = self.generate_base_map()
        
        self.publish_map()
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info("ERC Map Generator initialized (400x400 grid).")

    def odom_callback(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        
        # Trigger dynamic wall if the robot has moved to the middle (e.g. x > 8.0)
        if not self.wall_triggered and self.rx >= 8.0:
            self.wall_triggered = True
            self.get_logger().info("ERC Dynamic barrier triggered!")
            self.publish_map()

    def generate_base_map(self):
        # 400x400 cells. Initialized to 0.
        data = np.zeros((self.height, self.width), dtype=np.int8)
        
        # Add some static obstacles (canyons/walls)
        # Wall 1: horizontal wall blocking the lower middle
        # y from 150 to 160 (~7.5m to 8.0m), x from 0 to 280 (~14.0m)
        data[150:160, 0:280] = 100
        
        # Wall 2: horizontal wall blocking the upper middle
        # y from 250 to 260 (~12.5m to 13.0m), x from 120 to 400
        data[250:260, 120:400] = 100
        
        # Add continuous rough patches (cost 50) using simple geometric regions
        # Rough patch 1: centered at (15.0m, 5.0m) -> cells x[250:350], y[70:130]
        data[70:130, 250:350] = np.maximum(data[70:130, 250:350], 50)
        
        # Rough patch 2: centered at (5.0m, 15.0m) -> cells x[50:150], y[270:330]
        data[270:330, 50:150] = np.maximum(data[270:330, 50:150], 50)
        
        # Let's flatten to list
        return data.flatten().tolist()

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Copy base data
        data = list(self.base_data)
        
        if self.wall_triggered:
            # Dynamic wall that blocks the opening of Wall 1
            # Wall 1 opening was at x > 280. We block x from 280 to 320 at y=150 to 160
            for y in range(150, 160):
                for x in range(280, 320):
                    data[y * self.width + x] = 100
                    
        msg.data = data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ErcMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
