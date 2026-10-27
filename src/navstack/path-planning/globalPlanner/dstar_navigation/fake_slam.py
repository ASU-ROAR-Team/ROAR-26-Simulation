#!/usr/bin/env python3

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = [0.0]*4
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class FakeSlamNode(Node):
    def __init__(self):
        super().__init__('fake_slam')
        
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        
        rate = self.get_parameter('publish_rate').value
        self.dt = 1.0 / rate if rate > 0 else 0.05
        
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos)
        self.heightmap_pub = self.create_publisher(OccupancyGrid, '/active_map/heightmap', qos)
        self.range_pub = self.create_publisher(Float32MultiArray, '/active_map/heightmap_range', qos)
        self.marker_pub = self.create_publisher(Marker, '/robot_marker', 10)
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.create_maps()

    def create_maps(self):
        self.grid_msg = OccupancyGrid()
        self.grid_msg.header.frame_id = 'map'
        self.grid_msg.info.resolution = 0.1
        self.grid_msg.info.width = 200
        self.grid_msg.info.height = 200
        self.grid_msg.info.origin.position.x = 0.0
        self.grid_msg.info.origin.position.y = 0.0
        self.grid_msg.info.origin.orientation.w = 1.0
        
        map_data = [0] * (200 * 200)
        
        # Wall 1: x=[60,65], y=[30,120]
        for i in range(60, 66):
            for j in range(30, 121):
                map_data[j * 200 + i] = 100
                
        # Wall 2: x=[120,125], y=[80,170]
        for i in range(120, 126):
            for j in range(80, 171):
                map_data[j * 200 + i] = 100
                
        # Wall 3: x=[60,120], y=[30,35]
        for i in range(60, 121):
            for j in range(30, 36):
                map_data[j * 200 + i] = 100
                
        self.grid_msg.data = map_data
        
        self.hmap_msg = OccupancyGrid()
        self.hmap_msg.header.frame_id = 'map'
        # Copy info fields individually (avoid shared reference with grid_msg.info)
        self.hmap_msg.info.resolution = self.grid_msg.info.resolution
        self.hmap_msg.info.width = self.grid_msg.info.width
        self.hmap_msg.info.height = self.grid_msg.info.height
        self.hmap_msg.info.origin.position.x = self.grid_msg.info.origin.position.x
        self.hmap_msg.info.origin.position.y = self.grid_msg.info.origin.position.y
        self.hmap_msg.info.origin.orientation.w = 1.0
        
        hmap_data = [10] * (200 * 200)
        
        # slope region: x=[140,180], y=[40,100]
        for i in range(140, 181):
            for j in range(40, 101):
                val = 10 + int(70 * (i - 140) / 40.0)
                hmap_data[j * 200 + i] = min(80, max(10, val))
                
        # rough region: x=[20,50], y=[120,160]
        for i in range(20, 51):
            for j in range(120, 161):
                hmap_data[j * 200 + i] = random.randint(10, 60)
                
        self.hmap_msg.data = hmap_data

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def timer_callback(self):
        now = self.get_clock().now()
        
        self.yaw += self.w * self.dt
        self.x += self.v * math.cos(self.yaw) * self.dt
        self.y += self.v * math.sin(self.yaw) * self.dt
        
        q = quaternion_from_euler(0, 0, self.yaw)
        
        # map -> odom
        t1 = TransformStamped()
        t1.header.stamp = now.to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.rotation.w = 1.0
        
        # odom -> base_footprint
        t2 = TransformStamped()
        t2.header.stamp = now.to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_footprint'
        t2.transform.translation.x = self.x
        t2.transform.translation.y = self.y
        t2.transform.rotation.x = q[0]
        t2.transform.rotation.y = q[1]
        t2.transform.rotation.z = q[2]
        t2.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform([t1, t2])
        
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'map' # usually odom, but for simplicity
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)
        
        self.grid_msg.header.stamp = now.to_msg()
        self.map_pub.publish(self.grid_msg)
        
        self.hmap_msg.header.stamp = now.to_msg()
        self.heightmap_pub.publish(self.hmap_msg)
        
        range_msg = Float32MultiArray()
        range_msg.data = [0.0, 2.0]
        self.range_pub.publish(range_msg)
        
        marker = Marker()
        marker.header.stamp = now.to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'robot'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.25
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = FakeSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
