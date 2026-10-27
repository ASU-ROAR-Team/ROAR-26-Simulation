#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # Maps usually need a "transient local" QoS so late subscribers still get the map
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        self.timer = self.create_timer(2.0, self.publish_map)
        self.get_logger().info("Map Publisher initialized.")

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.info.resolution = 0.05
        msg.info.width = 100
        msg.info.height = 100
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Initialize map with 0 (free space)
        data = [0] * (msg.info.width * msg.info.height)
        
        # Create a U-shaped wall (100 = lethal)
        # Left wall
        for y in range(20, 80):
            data[y * msg.info.width + 40] = 100
        # Bottom wall
        for x in range(40, 60):
            data[20 * msg.info.width + x] = 100
        # Right wall
        for y in range(20, 80):
            data[y * msg.info.width + 60] = 100

        msg.data = data
        self.publisher_.publish(msg)    

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()