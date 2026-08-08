#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class PathSimulator(Node):
    def __init__(self):
        super().__init__('path_simulator')
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_sub = self.create_subscription(Path, '/global_plan', self.path_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Declare parameters
        self.declare_parameter('start_x', 0.5)
        self.declare_parameter('start_y', 0.5)
        self.declare_parameter('speed', 0.2)
        
        # Simulation state
        self.current_x = self.get_parameter('start_x').get_parameter_value().double_value
        self.current_y = self.get_parameter('start_y').get_parameter_value().double_value
        self.current_yaw = 0.0
        self.path = []
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        
        # 20 Hz control loop
        self.timer_period = 0.05
        self.timer = self.create_timer(self.timer_period, self.update_position)
        
        self.get_logger().info(f"Simulator initialized at x:{self.current_x:.2f}, y:{self.current_y:.2f}")
        self.raw_poses = []

    def path_callback(self, msg):
        self.get_logger().info(f"Received new plan with {len(msg.poses)} points.")
        self.raw_poses = msg.poses

    def update_position(self):
        if self.raw_poses:
            best_dist = float('inf')
            closest_idx = 0
            for i, p in enumerate(self.raw_poses):
                d = math.hypot(p.pose.position.x - self.current_x, p.pose.position.y - self.current_y)
                if d < best_dist:
                    best_dist = d
                    closest_idx = i
            
            lookahead_idx = min(closest_idx + 3, len(self.raw_poses) - 1)
            target = self.raw_poses[lookahead_idx].pose.position
            dx = target.x - self.current_x
            dy = target.y - self.current_y
            distance = math.hypot(dx, dy)
            if distance > 0.005:
                self.current_yaw = math.atan2(dy, dx)
                move_dist = min(self.speed * self.timer_period, distance)
                self.current_x += move_dist * math.cos(self.current_yaw)
                self.current_y += move_dist * math.sin(self.current_yaw)

        # Publish Odom and TF
        now = self.get_clock().now().to_msg()
        
        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.current_x
        odom.pose.pose.position.y = self.current_y
        
        # Simple Euler to Quaternion for Yaw
        odom.pose.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        self.odom_pub.publish(odom)
        
        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PathSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()