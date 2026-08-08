#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy
import math

class DynamicMapPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_map_publisher')
        
        # Parameters
        self.declare_parameter('scenario', 'surprise_wall')
        self.declare_parameter('width', 100)
        self.declare_parameter('height', 100)
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('new_goal_x', 4.5)
        self.declare_parameter('new_goal_y', 0.5)
        
        self.scenario = self.get_parameter('scenario').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.start_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.start_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.new_goal_x = self.get_parameter('new_goal_x').get_parameter_value().double_value
        self.new_goal_y = self.get_parameter('new_goal_y').get_parameter_value().double_value
        
        # Robot position tracking
        self.rx = self.start_x
        self.ry = self.start_y
        self.ryaw = 0.0
        self.has_odom = False
        
        # Persistent revealed map for LiDAR scenarios
        self.revealed_map = [-1] * (self.width * self.height)
        
        # Scenario state variables
        self.wall1_active = False
        self.wall2_active = False
        self.wall3_active = False
        
        # QoS for map publisher (matching TRANSIENT_LOCAL)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # Subscribe to Odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Publish initial map
        self.publish_map()
        
        self.get_logger().info(f"Dynamic Map Publisher initialized for scenario: {self.scenario}")
        
        # For simulated_lidar, lidar_maze_120 and lidar_unreachable_goal: re-publish on robot movement
        if self.scenario in ['simulated_lidar', 'lidar_maze_120', 'lidar_unreachable_goal']:
            self.last_publish_pos = (self.start_x, self.start_y)
            self.timer = self.create_timer(0.2, self._lidar_timer_cb)

    def odom_callback(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        self.has_odom = True
        
        # Extract orientation yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.ryaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Check triggers based on distance from start position
        dist = math.hypot(self.rx - self.start_x, self.ry - self.start_y)
        
        if self.scenario == 'known_maze_blocked':
            # Trigger wall blockage when robot reaches x >= 2.5
            if self.rx >= 2.5 and not self.wall1_active:
                self.wall1_active = True
                self.get_logger().info("Known maze blocked wall triggered!")
                self.publish_map()
        
        if self.scenario == 'surprise_wall':
            if dist >= 1.0 and not self.wall1_active:
                self.wall1_active = True
                self.get_logger().info("Surprise wall triggered!")
                self.publish_map()
                
        elif self.scenario == 'sequential_walls':
            if dist >= 0.8 and not self.wall1_active:
                self.wall1_active = True
                self.get_logger().info("Sequential Wall 1 triggered!")
                self.publish_map()
            if dist >= 1.8 and not self.wall2_active:
                self.wall2_active = True
                self.get_logger().info("Sequential Wall 2 triggered!")
                self.publish_map()
            if dist >= 2.8 and not self.wall3_active:
                self.wall3_active = True
                self.get_logger().info("Sequential Wall 3 triggered!")
                self.publish_map()

        elif self.scenario == 'corridor_block':
            if dist >= 1.5 and not self.wall1_active:
                self.wall1_active = True
                self.get_logger().info("Corridor block triggered!")
                self.publish_map()

        elif self.scenario == 'goal_change_dynamic':
            if dist >= 1.0 and not self.wall1_active:
                self.wall1_active = True
                self.get_logger().info("Dynamic wall triggered for goal_change scenario!")
                self.publish_map()

    def _lidar_timer_cb(self):
        """Re-publish map only when robot has moved enough to change the revealed area."""
        if not self.has_odom:
            return
        dx = self.rx - self.last_publish_pos[0]
        dy = self.ry - self.last_publish_pos[1]
        # Only republish if moved more than half a cell (resolution/2)
        if math.hypot(dx, dy) >= self.resolution / 2.0:
            self.last_publish_pos = (self.rx, self.ry)
            self.publish_map()

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
        
        # Handle maps depending on the scenario
        if self.scenario == 'known_maze_blocked':
            data = [0] * (self.width * self.height)
            # Middle divider wall: y from 50 to 60, x from 20 to 80
            for y in range(50, 60):
                for x in range(20, 80):
                    data[y * self.width + x] = 100
            # Bottom corridor outer boundary: y from 0 to 20, x from 20 to 80
            for y in range(0, 20):
                for x in range(20, 80):
                    data[y * self.width + x] = 100
            # Top corridor outer boundary: y from 90 to 100, x from 20 to 80
            for y in range(90, 100):
                for x in range(20, 80):
                    data[y * self.width + x] = 100
            # If triggered, block the bottom corridor at x = 65
            if self.wall1_active:
                for y in range(20, 50):
                    for x in range(63, 67):
                        data[y * self.width + x] = 100

        elif self.scenario == 'lidar_maze_120':
            # Ground truth S-shape maze walls
            hidden_wall = set()
            # Wall 1: x_grid = 35, y_grid from 0 to 70
            for y in range(0, 70):
                for x in range(33, 37):
                    hidden_wall.add(y * self.width + x)
            # Wall 2: x_grid = 70, y_grid from 30 to 100
            for y in range(30, 100):
                for x in range(68, 72):
                    hidden_wall.add(y * self.width + x)
            
            # Reveal radius in grid cells (2.0 meters radius for clear maze scanning)
            reveal_r = int(2.0 / self.resolution)
            
            # Find current robot grid position
            rc = int(self.rx / self.resolution)
            rr = int(self.ry / self.resolution)
            
            # Reveal cells inside the 120-degree frontal cone
            for r in range(max(0, rr - reveal_r), min(self.height, rr + reveal_r + 1)):
                for c in range(max(0, rc - reveal_r), min(self.width, rc + reveal_r + 1)):
                    idx = r * self.width + c
                    x_c = c * self.resolution + self.resolution / 2.0
                    y_c = r * self.resolution + self.resolution / 2.0
                    d = math.hypot(x_c - self.rx, y_c - self.ry)
                    
                    if d <= 2.0:
                        # Angle to cell
                        theta = math.atan2(y_c - self.ry, x_c - self.rx)
                        # Angular difference to robot's yaw heading
                        diff = math.atan2(math.sin(theta - self.ryaw), math.cos(theta - self.ryaw))
                        
                        # 120 degrees frontal FOV (60 degrees on either side of yaw)
                        if abs(diff) <= (math.pi / 3.0):
                            if idx in hidden_wall:
                                self.revealed_map[idx] = 100  # lethal
                            else:
                                self.revealed_map[idx] = 0    # free
            data = list(self.revealed_map)

        elif self.scenario == 'lidar_unreachable_goal':
            # Initialize with NO_INFORMATION if not already done
            # Hidden box enclosing the goal at (4.5, 4.5) -> grid cells (90, 90)
            hidden_obstacles = set()
            goal_gc_x = int(4.5 / self.resolution)
            goal_gc_y = int(4.5 / self.resolution)
            for y in range(goal_gc_y - 4, goal_gc_y + 5):
                for x in range(goal_gc_x - 4, goal_gc_x + 5):
                    hidden_obstacles.add(y * self.width + x)
            
            # Reveal radius of 1.5m
            reveal_r = int(1.5 / self.resolution)
            rc = int(self.rx / self.resolution)
            rr = int(self.ry / self.resolution)
            
            # Reveal start position initially
            start_rc = int(self.start_x / self.resolution)
            start_rr = int(self.start_y / self.resolution)
            for r in range(max(0, start_rr - reveal_r), min(self.height, start_rr + reveal_r + 1)):
                for c in range(max(0, start_rc - reveal_r), min(self.width, start_rc + reveal_r + 1)):
                    idx = r * self.width + c
                    if math.hypot(c - start_rc, r - start_rr) <= reveal_r:
                        self.revealed_map[idx] = 0
            
            # Reveal current local area
            for r in range(max(0, rr - reveal_r), min(self.height, rr + reveal_r + 1)):
                for c in range(max(0, rc - reveal_r), min(self.width, rc + reveal_r + 1)):
                    idx = r * self.width + c
                    if math.hypot(c - rc, r - rr) <= reveal_r:
                        if idx in hidden_obstacles:
                            self.revealed_map[idx] = 100  # lethal
                        else:
                            self.revealed_map[idx] = 0    # free
            data = list(self.revealed_map)

        elif self.scenario == 'surprise_wall':
            # Starts clear (0). When triggered, a wall appears at y_grid = 50, x_grid from 20 to 80.
            data = [0] * (self.width * self.height)
            if self.wall1_active:
                y_wall = 50
                for x in range(20, 80):
                    data[y_wall * self.width + x] = 100
                    
        elif self.scenario == 'sequential_walls':
            # Starts clear (0). Injects walls at sequential steps.
            data = [0] * (self.width * self.height)
            if self.wall1_active:
                # Wall 1 at y = 30, x from 0 to 70
                for x in range(0, 70):
                    data[30 * self.width + x] = 100
            if self.wall2_active:
                # Wall 2 at y = 60, x from 30 to 100
                for x in range(30, 100):
                    data[60 * self.width + x] = 100
            if self.wall3_active:
                # Wall 3 at y = 80, x from 0 to 75
                for x in range(0, 75):
                    data[80 * self.width + x] = 100
                    
        elif self.scenario == 'simulated_lidar':
            # Initialize with NO_INFORMATION (255 / -1 in signed char, published as 255 unsigned)
            data = [-1] * (self.width * self.height)
            
            # Static hidden obstacle setup
            # Wall at y = 50, x from 20 to 80
            hidden_wall = set()
            y_wall = 50
            for x in range(20, 80):
                hidden_wall.add(y_wall * self.width + x)
            
            # Reveal radius in grid cells
            reveal_r = int(1.5 / self.resolution) # 1.5 meters radius
            
            # Find current robot grid position
            rc = int(self.rx / self.resolution)
            rr = int(self.ry / self.resolution)
            
            # Reveal cells inside the circle around the robot
            for r in range(max(0, rr - reveal_r), min(self.height, rr + reveal_r + 1)):
                for c in range(max(0, rc - reveal_r), min(self.width, rc + reveal_r + 1)):
                    idx = r * self.width + c
                    if math.hypot(c - rc, r - rr) <= reveal_r:
                        if idx in hidden_wall:
                            data[idx] = 100 # lethal
                        else:
                            data[idx] = 0 # free space
                            
        elif self.scenario == 'corridor_block':
            # Flat map with a horizontal corridor at y=40-60 from x=20 to x=80.
            # When wall1_active: a barrier blocks the corridor at x=60.
            data = [0] * (self.width * self.height)
            # Top corridor wall
            for x in range(20, 80):
                data[40 * self.width + x] = 100
            # Bottom corridor wall
            for x in range(20, 80):
                data[60 * self.width + x] = 100
            if self.wall1_active:
                # Block the corridor at x=60
                for y in range(41, 60):
                    data[y * self.width + 60] = 100

        elif self.scenario == 'goal_change_dynamic':
            # Open map. When wall1_active: a vertical barrier appears at x=50.
            # The test harness will also issue a new goal mid-navigation.
            data = [0] * (self.width * self.height)
            if self.wall1_active:
                for y in range(20, 80):
                    data[y * self.width + 50] = 100

        else:
            self.get_logger().error(f"Unknown dynamic scenario: {self.scenario}")
            data = [0] * (self.width * self.height)

            
        msg.data = data
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
