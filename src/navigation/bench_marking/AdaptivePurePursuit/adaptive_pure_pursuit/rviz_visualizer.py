#!/usr/bin/env python3
"""RViz visualization node for path-tracking controller testing."""

import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker


class RVizVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__('rviz_visualizer_node')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.path_sub = self.create_subscription(Path, '/path', self.path_cb, self.qos_profile)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, self.qos_profile)

        self.ideal_path_pub = self.create_publisher(Path, '/visualization/ideal_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/visualization/actual_trajectory', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization/lookahead_marker', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.publish_visualization)

        self.latest_path: Optional[Path] = None
        self.latest_path_signature: Optional[Tuple[Tuple[float, float], ...]] = None
        self.latest_odom: Optional[Odometry] = None
        self.actual_points: List[Tuple[float, float]] = []
        self.max_actual_points = 400

        self.get_logger().info('RViz visualizer started.')

    def path_cb(self, msg: Path) -> None:
        path_signature = tuple(
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        )
        if self.latest_path_signature is not None and path_signature != self.latest_path_signature:
            self.get_logger().info('New test case path detected, clearing actual trajectory overlay.')
            self.actual_points = []

        self.latest_path_signature = path_signature
        self.latest_path = msg

    def odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        if self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            self.actual_points.append((pose.position.x, pose.position.y))
            if len(self.actual_points) > self.max_actual_points:
                self.actual_points = self.actual_points[-self.max_actual_points:]

    def publish_visualization(self) -> None:
        if self.latest_path is not None:
            self.ideal_path_pub.publish(self._build_path_message(self.latest_path.poses, frame_id='odom'))

        if self.actual_points:
            self.actual_path_pub.publish(self._build_path_message_from_points(self.actual_points))

        if self.latest_odom is not None:
            self.marker_pub.publish(self._build_lookahead_marker())

    def _build_path_message(self, poses: List[PoseStamped], frame_id: str = 'odom') -> Path:
        path = Path()
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        path.header.frame_id = frame_id
        path.poses = []
        for pose in poses:
            stamped_pose = PoseStamped()
            stamped_pose.header.stamp = now
            stamped_pose.header.frame_id = frame_id
            stamped_pose.pose = pose.pose
            path.poses.append(stamped_pose)
        return path

    def _build_path_message_from_points(self, points: List[Tuple[float, float]]) -> Path:
        path = Path()
        now = self.get_clock().now().to_msg()
        path.header.stamp = now
        path.header.frame_id = 'odom'
        for x, y in points:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self._yaw_to_quaternion(0.0)
            path.poses.append(pose)
        return path

    def _build_lookahead_marker(self) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'odom'
        marker.ns = 'ghost_car'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 2.5
        marker.scale.y = 1.2
        marker.scale.z = 0.5
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.6
        marker.lifetime.sec = 0

        if self.latest_odom is None:
            return marker

        marker.pose = self.latest_odom.pose.pose
        marker.pose.position.z = 0.25
        return marker

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RVizVisualizerNode()
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
