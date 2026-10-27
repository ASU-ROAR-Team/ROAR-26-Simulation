#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

class ComprehensivePathPublisher(Node):
    def __init__(self):
        super().__init__('comprehensive_path_publisher')
        self.DS = 0.05 

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        self.qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, 
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)

        self.path_pub = self.create_publisher(Path, '/path', self.qos)
        self.global_pub = self.create_publisher(Path, '/global_path', self.qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', self.qos)
        
        self.timer = self.create_timer(2.0, self.publish_all)
        self.generate_data()
        
    def generate_data(self):
        x_segments, y_segments = [], []
        def append_seg(xs, ys): x_segments.append(xs); y_segments.append(ys)

        # 1. Straight Line (0,-2) to (0,10)
        y1 = np.linspace(-2.0, 10.0, int(12.0 / self.DS) + 1)
        append_seg(np.zeros_like(y1), y1)

        # 2. Sharp 90deg Turn to (10,10)
        x2 = np.linspace(0.0, 10.0, int(10.0 / self.DS) + 1)
        append_seg(x2, np.full_like(x2, 10.0))
        
        # 3. Circular Arc to (10,0)
        R, cx, cy = 5.0, 10.0, 5.0
        theta = np.linspace(math.pi/2, -math.pi/2, int(math.pi*R/self.DS) + 1)
        append_seg(cx + R*np.cos(theta), cy + R*np.sin(theta))

        self.xs = np.concatenate(x_segments); self.ys = np.concatenate(y_segments)
        
        # Build Path Message
        self.path_msg = Path(); self.path_msg.header.frame_id = "odom"
        for x, y in zip(self.xs, self.ys):
            p = PoseStamped(); p.pose.position.x = float(x); p.pose.position.y = float(y)
            self.path_msg.poses.append(p)

    def publish_all(self):
        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        self.path_pub.publish(self.path_msg)
        self.global_pub.publish(self.path_msg) # Show same path in Green for reference

        # Markers for Start and Goal
        ma = MarkerArray()
        for i, (x, y, label, color) in enumerate([(0.0, -2.0, "START", (0.0, 1.0, 0.0)), 
                                                  (self.xs[-1], self.ys[-1], "GOAL", (1.0, 0.0, 0.0))]):
            m = Marker(); m.header.frame_id = "odom"; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = x; m.pose.position.y = y; m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = color[0], color[1], color[2], 1.0
            ma.markers.append(m)
        self.marker_pub.publish(ma)

def main():
    rclpy.init(); rclpy.spin(ComprehensivePathPublisher()); rclpy.shutdown()

if __name__ == "__main__": main()
