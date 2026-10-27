#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

class PathFollowerNode(Node):
    def __init__(self):
        super().__init__('path_follower')
        
        self.declare_parameter('speed', 0.3)
        
        self.path = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal_reached = False
        
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(Path, '/global_plan', self.path_callback, qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def path_callback(self, msg):
        self.path = msg
        self.goal_reached = False
        self.get_logger().info('Received new path!')
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation)
        
    def get_distance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        
    def control_loop(self):
        if not self.path or len(self.path.poses) == 0:
            return
            
        if self.goal_reached:
            return
            
        speed = self.get_parameter('speed').value
        
        # Find closest point
        closest_idx = 0
        min_dist = float('inf')
        for i, pose in enumerate(self.path.poses):
            d = self.get_distance((self.x, self.y), (pose.pose.position.x, pose.pose.position.y))
            if d < min_dist:
                min_dist = d
                closest_idx = i
                
        lookahead_idx = min(closest_idx + 3, len(self.path.poses) - 1)
        target_pose = self.path.poses[lookahead_idx].pose.position
        
        goal_pose = self.path.poses[-1].pose.position
        dist_to_goal = self.get_distance((self.x, self.y), (goal_pose.x, goal_pose.y))
        
        cmd = Twist()
        if dist_to_goal < 0.3:
            self.get_logger().info('Goal reached!')
            self.goal_reached = True
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            dx = target_pose.x - self.x
            dy = target_pose.y - self.y
            target_heading = math.atan2(dy, dx)
            
            heading_err = target_heading - self.yaw
            # normalize
            while heading_err > math.pi: heading_err -= 2.0 * math.pi
            while heading_err < -math.pi: heading_err += 2.0 * math.pi
            
            cmd.linear.x = speed
            cmd.angular.z = max(-1.5, min(1.5, 2.0 * heading_err))  # clamp angular vel
            
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
