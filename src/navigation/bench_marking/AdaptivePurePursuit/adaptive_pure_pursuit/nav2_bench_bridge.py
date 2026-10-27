#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
import math


def _path_signature(msg: Path):
    return tuple(
        (
            round(p.pose.position.x, 6),
            round(p.pose.position.y, 6),
            round(p.pose.position.z, 6),
        )
        for p in msg.poses
    )


class Nav2BenchBridge(Node):
    """
    Bridge node to connect existing test_orchestrator output (/path and /odometry/filtered)
    to Nav2's controller_server (MPPI plugin).
    
    1. Broadcasts TF (odom -> base_footprint) from incoming /odometry/filtered messages.
    2. Sends received /path as a FollowPath action goal to controller_server.
    """
    def __init__(self):
        super().__init__('nav2_bench_bridge')

        self.odom_topic = self.declare_parameter('odom_topic', '/odometry/filtered').value
        self.path_topic = self.declare_parameter('path_topic', '/path').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value

        # Track whether a FollowPath goal is currently active so a new path can
        # replace the previous mission by cancelling it first.
        self._goal_in_progress = False
        self._active_goal_handle = None
        self._last_path_signature = None
        self._last_goal_succeeded = False
        self._active_goal_signature = None

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Odom subscriber -> broadcast TF
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

# Path subscriber -> Action Goal
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            path_qos
        )

        # Republish odometry on the topic Nav2 expects so controllers receive
        # the velocity feedback needed to compute trajectories.
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # FollowPath Action Client
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        self.get_logger().info("Nav2 Bench Bridge initialized. Waiting for follow_path action server...")

    def odom_callback(self, msg: Odometry):
        self.odom_pub.publish(msg)

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warn("Received empty path, skipping action goal.")
            return

        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("FollowPath action server not available!")
            return

        path_signature = _path_signature(msg)
        if self._goal_in_progress and self._active_goal_signature == path_signature:
            self.get_logger().debug("Skipping repeated path message for current active goal.")
            return

        if (
            self._last_path_signature is not None
            and self._last_path_signature == path_signature
            and not self._goal_in_progress
            and self._last_goal_succeeded
        ):
            self.get_logger().debug(
                "Skipping repeated path message after previous successful completion."
            )
            return

        self._last_path_signature = path_signature
        self._active_goal_signature = path_signature
        self._last_goal_succeeded = False

        if self._goal_in_progress and self._active_goal_handle is not None:
            self.get_logger().info("Cancelling previous FollowPath goal before sending a new one.")
            self._active_goal_handle.cancel_goal_async()

        goal_msg = FollowPath.Goal()
        goal_msg.path = msg
        goal_msg.controller_id = 'FollowPath'

        self._goal_in_progress = True
        self.get_logger().info(f"Sending Path with {len(msg.poses)} poses to FollowPath action server...")
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_in_progress = False
            self._active_goal_handle = None
            self._active_goal_signature = None
            self._last_path_signature = None
            self._last_goal_succeeded = False
            self.get_logger().warn("FollowPath goal rejected by controller_server.")
            return

        self._active_goal_handle = goal_handle
        self.get_logger().info("FollowPath goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"FollowPath finished with status: {result.status}")
        self._goal_in_progress = False
        self._active_goal_handle = None
        self._active_goal_signature = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._last_goal_succeeded = True
        else:
            self._last_path_signature = None
            self._last_goal_succeeded = False


def main(args=None):
    rclpy.init(args=args)
    node = Nav2BenchBridge()
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
