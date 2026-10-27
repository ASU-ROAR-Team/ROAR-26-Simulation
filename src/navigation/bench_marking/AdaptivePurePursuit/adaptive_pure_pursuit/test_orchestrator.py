#!/usr/bin/env python3
"""State-machine driven mock test orchestrator for AdaptiveLd_fixed.

This node is a synthetic benchmark driver: it generates a sequence of test paths
and odometry updates for the controller stack. The data is published on the
standard topics consumed by nav2_bench_bridge.py, which relays the path and
pose information to Nav2 as a FollowPath action goal.
"""

import math
import random
import time
from typing import Dict, List, Optional, Tuple
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def _resolve_case_deadline(global_timeout_s: float, case_duration_s: float) -> float:
    """Return the effective case deadline for the current mission."""
    return max(float(global_timeout_s), float(case_duration_s))


class TestOrchestratorNode(Node):
    """Publishes synthetic path and odometry for closed-loop controller evaluation."""

    def __init__(self) -> None:
        super().__init__('test_orchestrator_node')

        self.declare_parameter('test_case', 'all')
        self.declare_parameter('custom_path', '')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('global_timeout_s', 30.0)
        self.declare_parameter('custom_path_duration_s', 60.0)

        self.test_case_selection = self.get_parameter('test_case').value
        self.custom_path_selection = self.get_parameter('custom_path').value
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.global_timeout_s = float(self.get_parameter('global_timeout_s').value)
        self.custom_path_duration_s = float(self.get_parameter('custom_path_duration_s').value)
        self._tick_count = 0

        # Populated only when a custom_path is supplied; holds the parsed points
        # so _build_case('CUSTOM') doesn't have to re-parse the string every time.
        self.custom_case_points: List[Tuple[float, float]] = []

        self.add_on_set_parameters_callback(self.parameters_callback)

        # These publishers feed the Nav2 bridge. The bridge node subscribes to
        # /path and /odometry/filtered and forwards them to the controller server.
        path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.path_pub = self.create_publisher(Path, '/path', path_qos)
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.case_pub = self.create_publisher(String, '/testing/current_case', 10)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)

        self.case_ids = [
            'TC-01', 'TC-02', 'TC-03', 'TC-04', 'TC-05', 'TC-06',
            'TC-07', 'TC-08', 'TC-09', 'TC-10', 'TC-11', 'TC-12',
        ]
        self.case_queue: List[str] = []
        self.current_case_id: Optional[str] = None
        self.current_path_points: List[Tuple[float, float]] = []
        self.current_path_msg: Optional[Path] = None

        self.vehicle_pose: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.current_cmd = Twist()

        self.case_start_time: Optional[float] = None
        self.last_update_time: Optional[float] = None
        self.case_elapsed_s = 0.0
        self.completed_cases: List[str] = []
        self.finished = False
        self.dynamic_event_triggered = False
        self.end_reached_time: Optional[float] = None
        self.case_duration_s = 0.0

        self._initialize_case_queue()
        self.get_logger().info(f"Closed-loop orchestrator started. test_case={self.test_case_selection}")

    def parameters_callback(self, params: List[Parameter]) -> SetParametersResult:
        for param in params:
            if param.name == 'test_case' and isinstance(param.value, str):
                self.test_case_selection = param.value
                self.get_logger().info(f"Updated test_case to: {self.test_case_selection}")
                self._initialize_case_queue()
            elif param.name == 'custom_path' and isinstance(param.value, str):
                self.custom_path_selection = param.value
                self.get_logger().info(f"Updated custom_path to: {self.custom_path_selection}")
                self._initialize_case_queue()
            elif param.name == 'publish_rate_hz':
                self.publish_rate_hz = float(param.value)
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
            elif param.name == 'global_timeout_s':
                self.global_timeout_s = float(param.value)
        return SetParametersResult(successful=True)

    def cmd_callback(self, msg: Twist) -> None:
        self.current_cmd = msg

    def _initialize_case_queue(self) -> None:
        # Priority: an explicit custom_path always wins over test_case.
        if self.custom_path_selection.strip():
            if self.test_case_selection.strip().lower() not in ('', 'all'):
                self.get_logger().warn(
                    f"Both 'custom_path' and 'test' were supplied. 'custom_path' takes "
                    f"priority; ignoring test='{self.test_case_selection}'."
                )
            parsed_points = self._parse_custom_path(self.custom_path_selection)
            if parsed_points is None:
                # _parse_custom_path already logged the exact reason.
                self._fail_and_shutdown("Invalid custom_path. Aborting launch.")
                return
            self.custom_case_points = parsed_points
            self.case_queue = ['CUSTOM']
        else:
            if self.test_case_selection.strip().lower() in ('', 'all'):
                self.case_queue = list(self.case_ids)
            else:
                requested_ids = [t.strip().upper() for t in self.test_case_selection.split(',') if t.strip()]
                invalid_ids = [t for t in requested_ids if t not in self.case_ids]
                if invalid_ids:
                    self.get_logger().error(
                        f"Unknown test ID(s): {', '.join(invalid_ids)}. "
                        f"Valid test IDs are: {', '.join(self.case_ids)}."
                    )
                    self._fail_and_shutdown("Invalid test ID(s) in 'test' argument. Aborting launch.")
                    return
                self.case_queue = requested_ids

        self.current_case_id = None
        self.current_path_points = []
        self.current_path_msg = None
        self.vehicle_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.current_cmd = Twist()
        self.case_start_time = None
        self.case_elapsed_s = 0.0
        self.completed_cases = []
        self.finished = False
        self.dynamic_event_triggered = False
        self.end_reached_time = None
        self.case_duration_s = 0.0

        self._start_next_case()

    def _parse_custom_path(self, path_str: str) -> Optional[List[Tuple[float, float]]]:
        """Parse 'x1,y1;x2,y2;x3,y3' into a list of (x, y) points.

        Returns None (and logs the exact problem) if the string is malformed
        or has fewer than 2 points.
        """
        points: List[Tuple[float, float]] = []
        segments = [seg.strip() for seg in path_str.split(';') if seg.strip()]
        for seg in segments:
            parts = seg.split(',')
            if len(parts) != 2:
                self.get_logger().error(
                    f"custom_path segment '{seg}' is malformed. "
                    f"Expected 'x,y' pairs separated by ';', e.g. '0,0;1,0;2,0.5'."
                )
                return None
            try:
                x, y = float(parts[0].strip()), float(parts[1].strip())
            except ValueError:
                self.get_logger().error(
                    f"custom_path segment '{seg}' does not contain valid numbers."
                )
                return None
            points.append((x, y))

        if len(points) < 2:
            self.get_logger().error(
                f"custom_path must contain at least 2 points, got {len(points)}."
            )
            return None

        return points

    def _fail_and_shutdown(self, reason: str) -> None:
        """Log a fatal error and cleanly stop this node.

        This still ends the process, so the launch file's OnProcessExit chain
        (stop bag -> generate report -> shutdown) still runs as normal, just
        with whatever partial data (if any) was recorded.
        """
        self.get_logger().error(reason)
        self.finished = True
        if hasattr(self, 'timer'):
            self.timer.cancel()
        rclpy.shutdown()

    def _start_next_case(self) -> None:
        if not self.case_queue:
            self._finish_sequence()
            return

        self.current_case_id = self.case_queue.pop(0)
        self.current_path_points, initial_pose, duration_s = self._build_case(self.current_case_id)
        self._publish_case_id()
        self.current_path_msg = self._build_path_message(self.current_path_points)
        self.vehicle_pose = dict(initial_pose)
        self.current_cmd = Twist()
        self.dynamic_event_triggered = False
        self.end_reached_time = None
        self.case_duration_s = duration_s

        now = time.monotonic()
        self.case_start_time = now
        self.last_update_time = now
        self.case_elapsed_s = 0.0

        self.get_logger().info(f"Starting {self.current_case_id}")
        self._publish_path_message()

    def _update_kinematics(self) -> None:
        now = time.monotonic()
        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = now - self.last_update_time
        self.last_update_time = now

        v = self.current_cmd.linear.x
        omega = self.current_cmd.angular.z

        self.vehicle_pose['x'] += v * math.cos(self.vehicle_pose['yaw']) * dt
        self.vehicle_pose['y'] += v * math.sin(self.vehicle_pose['yaw']) * dt
        self.vehicle_pose['yaw'] += omega * dt

    def timer_callback(self) -> None:
        if self.finished:
            return

        if self.current_case_id is None:
            self._start_next_case()
            return

        self._update_kinematics()
        self.case_elapsed_s = time.monotonic() - self.case_start_time

        if self.current_case_id == 'TC-10' and self.case_elapsed_s >= 4.0 and not self.dynamic_event_triggered:
            new_points = [
                (p[0], 2.0) if p[0] > self.vehicle_pose['x'] else p for p in self.current_path_points
            ]
            self.current_path_points = new_points
            self.current_path_msg = self._build_path_message(self.current_path_points)
            self.dynamic_event_triggered = True
            self._publish_path_message()
            self.get_logger().warn("TC-10: Phantom obstacle detected! Path instantly shifted.")

        if self.current_path_points and self.end_reached_time is None:
            min_dist_sq = float('inf')
            closest_idx = 0
            for idx, p in enumerate(self.current_path_points):
                dist_sq = (self.vehicle_pose['x'] - p[0]) ** 2 + (self.vehicle_pose['y'] - p[1]) ** 2
                if dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    closest_idx = idx

            progress_ratio = closest_idx / max(1, len(self.current_path_points) - 1)
            final_x, final_y = self.current_path_points[-1]
            dist_to_final = math.hypot(self.vehicle_pose['x'] - final_x, self.vehicle_pose['y'] - final_y)

            if dist_to_final <= 0.5 and progress_ratio >= 0.85:
                self.end_reached_time = time.monotonic()
                self.get_logger().info("Target reached! Observing for overshoot...")

        time_to_end = False
        reason = ""

        deadline_s = _resolve_case_deadline(self.global_timeout_s, self.case_duration_s)

        if self.end_reached_time is not None:
            time_to_end = True
            reason = "Reached final point"
        elif self.case_elapsed_s >= deadline_s:
            time_to_end = True
            reason = f"Case deadline met ({deadline_s:.1f}s; global_timeout={self.global_timeout_s:.1f}s, case_duration={self.case_duration_s:.1f}s)"

        if time_to_end:
            self.completed_cases.append(self.current_case_id)
            self.get_logger().info(
                f"Completed {self.current_case_id} ({reason} at {self.case_elapsed_s:.1f}s)"
            )
            self._start_next_case()
            return

        self._tick_count += 1
        if self._tick_count % max(1, int(self.publish_rate_hz)) == 0:
            self._publish_path_message()

        self._publish_current_state()

    def _publish_case_id(self) -> None:
        if self.current_case_id is None:
            return
        msg = String()
        msg.data = self.current_case_id
        self.case_pub.publish(msg)

    def _refresh_path_timestamps(self) -> None:
        if self.current_path_msg is None:
            return

        now = self.get_clock().now().to_msg()
        self.current_path_msg.header.stamp = now
        for pose in self.current_path_msg.poses:
            pose.header.stamp = now

    def _publish_path_message(self) -> None:
        if self.current_path_msg is None:
            return

        self._refresh_path_timestamps()
        self._publish_case_id()
        self.path_pub.publish(self.current_path_msg)

    def _publish_current_state(self) -> None:
        if self.current_path_msg is None:
            return

        self._publish_case_id()

        odom_msg = self._build_odometry_message()
        self.odom_pub.publish(odom_msg)
        self._broadcast_tf(odom_msg)

    def _build_odometry_message(self) -> Odometry:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        noise_x = 0.0
        noise_y = 0.0
        if self.current_case_id == 'TC-08':
            noise_x = random.uniform(-0.15, 0.15)
            noise_y = random.uniform(-0.15, 0.15)

        odom.pose.pose.position.x = self.vehicle_pose['x'] + noise_x
        odom.pose.pose.position.y = self.vehicle_pose['y'] + noise_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self.vehicle_pose['yaw'])
        odom.twist.twist.linear.x = self.current_cmd.linear.x
        odom.twist.twist.angular.z = self.current_cmd.angular.z
        return odom

    def _build_path_message(self, points: List[Tuple[float, float]]) -> Path:
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

    def _broadcast_tf(self, odom: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def _build_case(self, case_id: str) -> Tuple[List[Tuple[float, float]], Dict[str, float], float]:
        points = []
        if case_id == 'CUSTOM':
            points = list(self.custom_case_points)
            first_x, first_y = points[0]
            if len(points) > 1:
                second_x, second_y = points[1]
                initial_yaw = math.atan2(second_y - first_y, second_x - first_x)
            else:
                initial_yaw = 0.0
            initial_pose = {'x': first_x, 'y': first_y, 'yaw': initial_yaw}
            duration_s = self.custom_path_duration_s
        elif case_id == 'TC-01':
            for i in range(150):
                x = i * 0.1
                y = 1.5 * math.sin(x * 0.8)
                points.append((x, y))
            initial_yaw = math.atan2(1.5 * 0.8 * math.cos(0.0), 1.0)
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': initial_yaw}
            duration_s = 40.0
        elif case_id == 'TC-02':
            for i in range(150):
                x = i * 0.1
                y = 0.0 if x < 5.0 else 2.0
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0
        elif case_id == 'TC-03':
            for i in range(100):
                x = -2.0 - (i * 0.1)
                y = 0.0
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0
        elif case_id == 'TC-04':
            for i in range(50):
                x = i * 0.1
                y = math.sin(x)
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 5.0
        elif case_id == 'TC-05':
            for i in range(150):
                x = i * 0.1
                y = math.atan(x - 7.5) + 1.5
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': math.atan(-7.5) + 1.5, 'yaw': 0.0}
            duration_s = 40.0
        elif case_id == 'TC-06':
            for i in range(150):
                x = i * 0.1
                y = 0.0
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 3.0, 'yaw': math.radians(45.0)}
            duration_s = 40.0
        elif case_id == 'TC-07':
            for i in range(150):
                x = i * 0.1
                y = 1.2 * math.sin(x * 1.5)
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': math.atan2(1.2 * 1.5 * math.cos(0.0), 1.0)}
            duration_s = 40.0
        elif case_id == 'TC-08':
            for i in range(150):
                x = i * 0.1
                y = 0.0
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 10.0
        elif case_id == 'TC-09':
            for i in range(50):
                x = i * 0.1
                points.append((x, 0.0))
            for i in range(31):
                theta = -math.pi / 2 + (i / 30.0) * math.pi
                x = 5.0 + 1.5 * math.cos(theta)
                y = 1.5 + 1.5 * math.sin(theta)
                points.append((x, y))
            for i in range(50):
                x = 5.0 - (i * 0.1)
                points.append((x, 3.0))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0
        elif case_id == 'TC-10':
            for i in range(150):
                x = i * 0.1
                y = 0.0
                points.append((x, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0
        elif case_id == 'TC-11':
            for i in range(200):
                t = (i / 200.0) * (2 * math.pi)
                x = 5.0 * math.sin(t)
                y = 2.5 * math.sin(t) * math.cos(t)
                points.append((x, y))
            initial_yaw = math.atan2(2.5, 5.0)
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': initial_yaw}
            duration_s = 40.0
        elif case_id == 'TC-12':
            for i in range(40):
                x = i * 0.1
                points.append((x, 0.0))
            for i in range(31):
                theta = -math.pi / 2 + (i / 30.0) * (math.pi / 2)
                x = 4.0 + 1.0 * math.cos(theta)
                y = -1.0 + 1.0 * math.sin(theta)
                points.append((x, y))
            for i in range(40):
                y = -1.0 - (i * 0.1)
                points.append((5.0, y))
            for i in range(45):
                theta = math.pi + (i / 44.0) * math.pi
                x = 7.0 + 2.0 * math.cos(theta)
                y = -5.0 + 2.0 * math.sin(theta)
                points.append((x, y))
            for i in range(40):
                y = -5.0 + (i * 0.1)
                points.append((9.0, y))
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0
        else:
            points = [(0.0, 0.0), (1.0, 0.0)]
            initial_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            duration_s = 40.0

        return points, initial_pose, duration_s

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _finish_sequence(self) -> None:
        # Mark the sequence finished and schedule a safe shutdown after
        # the current timer callback returns to avoid deadlocking the rclpy
        # executor (do NOT call rclpy.shutdown() here directly).
        self.finished = True
        self.get_logger().info(f"Completed test sequence: {self.completed_cases}")
        self.get_logger().info(
            "======== MPPI BENCHMARKING SEQUENCE FINISHED, REPORT GENERATED ========"
        )

        # Cancel the primary timer if present
        if hasattr(self, 'timer'):
            try:
                self.timer.cancel()
            except Exception:
                pass

        # Schedule a one-shot timer to perform node destruction and rclpy.shutdown()
        # after the current executor callback returns.
        try:
            # store reference so it can be cancelled/inspected if needed
            self.shutdown_timer = self.create_timer(0.1, self._shutdown_callback)
        except Exception:
            # If timer creation fails for any reason, fall back to immediate
            # but guarded shutdown to avoid raising inside executor callbacks.
            try:
                self.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def _shutdown_callback(self) -> None:
        # One-shot shutdown callback executed outside the original timer
        # callback context. Cancel the shutdown timer (idempotent), log
        # that the benchmark completed and request launch shutdown by
        # exiting the process. Exiting the process (os._exit) ensures the
        # launch system receives the OnProcessExit event and can perform
        # its configured stop/generate-report/shutdown actions.
        if hasattr(self, 'shutdown_timer'):
            try:
                self.shutdown_timer.cancel()
            except Exception:
                pass

        # Log benchmark completion and request launch shutdown
        try:
            self.get_logger().info('BENCHMARK COMPLETE')
            self.get_logger().info('REQUESTING LAUNCH SHUTDOWN')
        except Exception:
            pass

        # Request shutdown from a non-executor thread to avoid deadlock:
        # spawn a short-lived daemon thread that calls rclpy.shutdown().
        def _call_shutdown_delayed():
            # give the executor a moment to finish current callback
            time.sleep(0.05)
            try:
                rclpy.shutdown()
            except Exception:
                pass

        t = threading.Thread(target=_call_shutdown_delayed, daemon=True)
        t.start()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = TestOrchestratorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        # Ensure the executor process actually terminates after shutdown.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()