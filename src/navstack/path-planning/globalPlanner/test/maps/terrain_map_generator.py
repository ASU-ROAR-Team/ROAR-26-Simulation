#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy

class TerrainMapGenerator(Node):
    def __init__(self):
        super().__init__('terrain_map_generator')
        
        # Parameters
        self.declare_parameter('scenario', 'rough_patch')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.05)
        
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        
        # QoS for map publisher
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        
        # Publish map periodically
        self.publish_map()
        self.timer = self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info(f"Terrain Map Generator initialized for scenario: {self.scenario}")

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
        
        if self.scenario == 'rough_patch':
            # Central high-cost patch (cost = 60).
            # Placed between x_cell 40 to 70 (~2.0m to 3.5m) and y_cell 20 to 80 (~1.0m to 4.0m)
            # A free detour exists below (y_cell < 20) and above (y_cell > 80)
            for y in range(20, 80):
                for x in range(40, 70):
                    data[y * self.width + x] = 60
                    
        elif self.scenario == 'gradient_slope':
            # Cost increases linearly from y=0 (cost 0) to y=height (cost 80)
            # This should attract paths to hug the lower y region.
            for y in range(self.height):
                cost_val = int((y / self.height) * 80)
                for x in range(self.width):
                    data[y * self.width + x] = cost_val

        elif self.scenario == 'forced_rough':
            # Central rough patch (cost = 60)
            for y in range(20, 80):
                for x in range(40, 70):
                    data[y * self.width + x] = 60
            # Block bottom detour (lethal 100)
            for y in range(0, 20):
                for x in range(30, 80):
                    data[y * self.width + x] = 100
            # Block top detour (lethal 100)
            for y in range(80, self.height):
                for x in range(30, 80):
                    data[y * self.width + x] = 100

        elif self.scenario == 'perlin_terrain':
            # Synthesize realistic terrain height using multi-frequency sine/cosine noise
            # Cost scales up to 70.
            for y in range(self.height):
                for x in range(self.width):
                    # Multi-frequency smooth noise
                    val = 0.0
                    val += math.sin(x * 0.1) * math.cos(y * 0.1) * 0.5
                    val += math.sin(x * 0.25) * math.sin(y * 0.3) * 0.25
                    val += math.cos(x * 0.5) * math.cos(y * 0.5) * 0.15
                    val += math.sin(x * 1.0) * math.cos(y * 1.2) * 0.1
                    # Normalize to [0, 1] and scale to max cost of 70
                    norm_val = (val + 1.0) / 2.0
                    data[y * self.width + x] = int(norm_val * 70)

                    
        else:
            self.get_logger().error(f"Unknown terrain scenario: {self.scenario}")
            
        msg.data = data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
