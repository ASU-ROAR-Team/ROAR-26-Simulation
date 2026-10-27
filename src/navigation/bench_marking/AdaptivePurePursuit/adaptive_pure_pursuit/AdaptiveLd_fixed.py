#!/usr/bin/env python3
# pylint: disable=all
# mypy: ignore-errors
"""
Adaptive Pure Pursuit controller — corrected version.

Changes from submitted code:
  FIX 1  self.distLd initialised to self.KC (= 0.1) instead of 0.5.
          0.5 exceeds the maximum valid lookahead (KL·Vmax+KC = 0.4 m)
          and causes an overshooting first step.
  FIX 2  In-place turn branch restored for |theta| > π/2.
          Without it, when the goal is behind the robot theta ≈ ±π,
          sin(π) = 0 → k = 0 → ω = 0 and the robot drives straight
          away from the goal (Test 4 fails).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import numpy as np


def euler_from_quaternion(quat):
    x, y, z, w = quat
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Control(Node):
    """Adaptive pure pursuit controller"""

    def __init__(self):
        super().__init__("adaptive_controller_fixed")

        self.pose_topic = self.declare_parameter(
            "subscribing_topics/pose", "/odometry/filtered"
        ).value
        self.path_topic = self.declare_parameter(
            "subscribing_topics/path", "/path"
        ).value
        
        self.simulate_pose = self.declare_parameter("simulate_pose", True).value
        self.MAXVELOCITY = self.declare_parameter("robot_parameters/maxVelocity", 1.0).value
        self.WIDTH       = self.declare_parameter("robot_parameters/width",       0.973).value
        # Wheel diameter is 0.25 m → radius 0.125 m
        self.wheel_radius = self.declare_parameter("robot_parameters/wheel_radius", 0.125).value
        self.KL          = self.declare_parameter("algorithm_parameters/KL",      0.3).value
        self.KC          = self.declare_parameter("algorithm_parameters/KC",      0.1).value
        self.min_speed   = self.declare_parameter("robot_parameters/min_speed",   0.10).value
        self.min_lookahead = self.declare_parameter("robot_parameters/min_lookahead", 0.2).value
        self.max_lookahead = self.declare_parameter("robot_parameters/max_lookahead", 1.2).value
        self.localPath   = self.declare_parameter("algorithm_parameters/localPath", False).value
        self.log         = self.declare_parameter("meta_parameters/log",           False).value
        self.min_turn_radius      = self.declare_parameter("robot_parameters/min_turn_radius",      1.0).value
        self.min_turn_speed_ratio = self.declare_parameter("robot_parameters/min_turn_speed_ratio",0.4).value
        self.publish_wheel_velocities = self.declare_parameter("robot_parameters/publish_wheel_velocities", True).value
        self.cmd_vel_topic = self.declare_parameter("publishing_topics/cmd_vel", "/cmd_vel").value
        self.wheel_topic = self.declare_parameter("publishing_topics/wheel_velocities", "/wheel_velocities").value

        # Prevent publishing two incompatible types on the same topic
        if self.publish_wheel_velocities and self.wheel_topic == self.cmd_vel_topic:
            self.get_logger().warn(
                "wheel_velocities topic must not be the same as cmd_vel because Twist and Float32MultiArray are incompatible. "
                "Disabling wheel velocity publication."
            )
            self.publish_wheel_velocities = False

        # Standard QoS for high-frequency control data
        self.control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.cmd_pub  = self.create_publisher(Twist,  self.cmd_vel_topic, self.control_qos)
        self.odom_pub = self.create_publisher(Odometry, "/odometry/filtered", self.control_qos)
        self.test_pub = self.create_publisher(String, "/control_test", 10)
        if self.publish_wheel_velocities:
            self.wheel_pub = self.create_publisher(Float32MultiArray, self.wheel_topic, self.control_qos)

        if not self.simulate_pose:
            self.create_subscription(Odometry, self.pose_topic, self.updatePose,    self.control_qos)
        
        self.create_subscription(Path,     self.path_topic, self.pathCallback,  self.control_qos)

        self.indexLD         = 0
        self.currentPosition = [0.0, -2.0, math.pi / 2.0]   # x, y, yaw (facing +Y)
        self.pose_received   = False

        if self.simulate_pose:
            self.pose_received = True

        # FIX 1: initialise to KC, not 0.5
        # Maximum valid value is KL*Vmax+KC = 0.3*1.0+0.1 = 0.4 m.
        # Starting at 0.5 overshoots that ceiling on the very first tick.
        self.distLd = self.KC

        self.timer_period = 0.02
        self.timer        = self.create_timer(self.timer_period, self.purePursuit)
        self.waypoints    = []

    def pathCallback(self, msg: Path):
        self.waypoints = [
            (pose.pose.position.x, pose.pose.position.y) for pose in msg.poses
        ]
        if self.log:
            self.get_logger().info(f"got waypoints: {len(self.waypoints)}")

    def updatePose(self, msg: Odometry):
        if not self.pose_received:
            self.get_logger().info("First odom received!")
        self.currentPosition[0] = msg.pose.pose.position.x
        self.currentPosition[1] = msg.pose.pose.position.y
        orientation     = msg.pose.pose.orientation
        orientationList = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw       = euler_from_quaternion(orientationList)
        self.currentPosition[2] = yaw
        self.pose_received = True

    def setVelocity(self, k: float) -> float:
        velocity = self.MAXVELOCITY / (1.0 + abs(k))
        velocity = max(self.min_speed, min(self.MAXVELOCITY, velocity))
        goalPoint = self.waypoints[-1]
        distanceToGoal = np.linalg.norm(
            np.array((self.currentPosition[0], self.currentPosition[1]))
            - np.array(goalPoint)
        )
        if distanceToGoal < 0.2:
            return 0.0
        return float(velocity)

    def findLookaheadPoint(self, robotPosition: tuple):
        if not self.waypoints:
            return None

        robot_pt = np.array(robotPosition)
        dists = np.linalg.norm(np.array(self.waypoints) - robot_pt, axis=1)
        closest_idx = int(np.argmin(dists))

        for i in range(closest_idx, len(self.waypoints)):
            waypoint = self.waypoints[i]
            distanceToRobot = np.linalg.norm(np.array(waypoint) - robot_pt)
            if distanceToRobot >= self.distLd:
                self.indexLD = i
                return waypoint

        return self.waypoints[-1]

    def purePursuit(self):
        if not self.pose_received:
            self.get_logger().info(
                "Waiting for odom on '%s'..." % self.pose_topic,
                throttle_duration_sec=3.0,
            )
            return
        if len(self.waypoints) == 0:
            self.get_logger().info(
                "Waiting for waypoints on '%s'..." % self.path_topic,
                throttle_duration_sec=3.0,
            )
            return

        lookaheadPoint = self.findLookaheadPoint(
            (self.currentPosition[0], self.currentPosition[1])
        )
        if lookaheadPoint is None:
            self.get_logger().warn("Lookahead point is None!")
            return

        dx = lookaheadPoint[0] - self.currentPosition[0]
        dy = lookaheadPoint[1] - self.currentPosition[1]
        Ld = math.hypot(dx, dy) + 1e-6

        headingX = math.cos(self.currentPosition[2])
        headingY = math.sin(self.currentPosition[2])

        dot   = dx * headingX + dy * headingY
        det   = headingX * dy - headingY * dx
        theta = math.atan2(det, dot)
        
        # Use distLd (desired) instead of Ld (actual) to avoid 1/0 and for stability
        # Pure Pursuit: k = 2*sin(theta)/Ld
        k = 2.0 * math.sin(theta) / max(0.1, self.distLd)
        
        # Cap curvature to avoid extreme angular velocities
        k = max(-5.0, min(5.0, k))

        v = self.setVelocity(k)

        # In-place turn branch: recover when the lookahead goal lies behind the robot.
        # Pure pursuit curvature goes to zero near |theta| ≈ π, which can drive straight away
        # from the path instead of turning back toward the goal.
        if abs(theta) > (math.pi / 2):
            v = min(v, 0.4 * self.MAXVELOCITY)
            max_inplace_ang = 1.0
            angular = math.copysign(max_inplace_ang, theta)
            mode = "IN-PLACE"
        else:
            angular = v * k
            angular = max(min(angular, 1.5), -1.5)
            mode = "NORMAL"

        if self.log:
            self.get_logger().info(
                f"Ld_pt: ({lookaheadPoint[0]:.2f}, {lookaheadPoint[1]:.2f}), "
                f"Ld_dist: {math.hypot(dx, dy):.2f}, "
                f"theta: {math.degrees(theta):.1f}°, k: {k:.2f}, "
                f"v: {v:.2f}, w: {angular:.2f}, mode: {mode}",
                throttle_duration_sec=0.5
            )

        # Update distLd for next tick based on current velocity
        self.distLd = self.KL * v + self.KC
        self.distLd = max(self.min_lookahead, min(self.max_lookahead, self.distLd))

        tw = Twist()
        tw.linear.x   = float(v)
        tw.angular.z  = float(angular)
        self.cmd_pub.publish(tw)

        if self.publish_wheel_velocities:
            # Publish six wheel angular velocities (rad/s) as Float32MultiArray
            # Differential-drive kinematics:
            # wL = (v - omega * WIDTH/2) / r
            # wR = (v + omega * WIDTH/2) / r
            r = float(self.wheel_radius)
            w_left = (v - angular * (self.WIDTH / 2.0)) / max(1e-6, r)
            w_right = (v + angular * (self.WIDTH / 2.0)) / max(1e-6, r)

            wheel_msg = Float32MultiArray()
            wheel_msg.data = [float(w_left), float(w_left), float(w_left), float(w_right), float(w_right), float(w_right)]
            self.wheel_pub.publish(wheel_msg)

        if self.log:
            self.get_logger().info(f"Published cmd_vel: v={v:.2f}, w={angular:.2f}", throttle_duration_sec=1.0)

        test_msg      = String()
        yaw_deg       = math.degrees(self.currentPosition[2])
        direction     = "straight"
        if angular >  0.01: direction = "left"
        elif angular < -0.01: direction = "right"
        test_msg.data = (
            f"yaw: {yaw_deg:.1f}°, velocity: {v:.3f} m/s, "
            f"angular: {angular:.3f} rad/s, direction: {direction}"
        )
        self.test_pub.publish(test_msg)

        if self.simulate_pose:
            dt  = self.timer_period
            yaw = self.currentPosition[2]
            self.currentPosition[0] += v * math.cos(yaw) * dt
            self.currentPosition[1] += v * math.sin(yaw) * dt
            self.currentPosition[2] += angular * dt
            # Wrap yaw to [-pi, pi]
            self.currentPosition[2] = (self.currentPosition[2] + math.pi) % (2 * math.pi) - math.pi
            
            # Publish simulated odom
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = self.currentPosition[0]
            odom.pose.pose.position.y = self.currentPosition[1]
            # yaw to quaternion
            odom.pose.pose.orientation.z = math.sin(self.currentPosition[2] / 2.0)
            odom.pose.pose.orientation.w = math.cos(self.currentPosition[2] / 2.0)
            self.odom_pub.publish(odom)

            if self.log:
                self.get_logger().info(
                    f"Sim pose: x={self.currentPosition[0]:.3f}, "
                    f"y={self.currentPosition[1]:.3f}, "
                    f"yaw={math.degrees(self.currentPosition[2]):.1f}°",
                    throttle_duration_sec=0.5,
                )


def main(args=None):
    rclpy.init(args=args)
    control_node = Control()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        if rclpy.ok():          # <--- Add this safety check
            rclpy.shutdown()


if __name__ == "__main__":
    main()
