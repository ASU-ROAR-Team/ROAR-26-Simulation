#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class TestMapPublisher(Node):
    def __init__(self):
        super().__init__('test_map_publisher')
        
        # Parameters
        self.declare_parameter('scenario', 'wall')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.05)
        
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        
        # QoS for late subscribers (matching TRANSIENT_LOCAL)
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        # Publish immediately, and periodically
        self.publish_map()
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(f"Test Map Publisher initialized with scenario: {self.scenario}")

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
        
        # Initialize map with 0 (free space)
        data = [0] * (self.width * self.height)
        
        if self.scenario == 'wall':
            # Horizontal wall blocking direct diagonal path from (0.5, 0.5) to (4.5, 4.5)
            # Wall at y = 50, x from 20 to 80
            y_wall = 50
            for x in range(20, 80):
                data[y_wall * self.width + x] = 100
                
        elif self.scenario == 'u_trap':
            # U-shaped wall open at the top (y > 70)
            # Start is inside at (2.5, 2.0), goal is outside at (2.5, 4.5)
            # Bottom wall
            for x in range(30, 70):
                data[30 * self.width + x] = 100
            # Left wall
            for y in range(30, 70):
                data[y * self.width + 30] = 100
            # Right wall
            for y in range(30, 70):
                data[y * self.width + 70] = 100
                
        elif self.scenario == 'corridor':
            # Narrow corridor of width 6 cells in the middle (x from 47 to 52)
            # Left block
            for y in range(0, self.height):
                for x in range(0, 47):
                    data[y * self.width + x] = 100
            # Right block
            for y in range(0, self.height):
                for x in range(53, self.width):
                    data[y * self.width + x] = 100
                    
        elif self.scenario == 'enclosed':
            # Start position is at (1.0, 1.0) -> index (20, 20)
            # Wall enclosing the start region
            for x in range(5, 36):
                data[35 * self.width + x] = 100
                data[5 * self.width + x] = 100
            for y in range(5, 36):
                data[y * self.width + 5] = 100
                data[y * self.width + 35] = 100
                
        elif self.scenario == 'open':
            # Keep as all zeros
            pass
            
        else:
            self.get_logger().error(f"Unknown scenario: {self.scenario}")

        msg.data = data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TestMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
