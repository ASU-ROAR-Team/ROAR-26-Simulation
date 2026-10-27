import os
import glob
import numpy as np
import rclpy
import json
import math
from pathlib import Path
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from referee_engine.critical_rule_enforcer import CriticalRuleEnforcer
from referee_engine.waypoint_tracker import WaypointTracker, Waypoint, load_waypoints_from_yaml
from referee_engine.score_calculator import ScoreCalculator, ScoreCalculatorConfig

class RefereeCoreNode(Node):
    """
    Automated Referee Core Node for ERC Navigation Task.
    """

    def __init__(self):
        super().__init__('referee_core_node')

        # Declare ROS parameters
        self.declare_parameter('timeout_duration_sec', 1200.0)
        self.declare_parameter('min_speed_m_s', 0.2)
        self.declare_parameter('max_speed_m_s', 1.0)
        self.declare_parameter('rover_radius_m', 0.5)
        self.declare_parameter('auto_advance', True)
        self.declare_parameter('only_gnss_used', False)
        self.declare_parameter('mission_directory', '')
        self.declare_parameter('mission_name', 'mission00')
        self.declare_parameter('yaml_params_file', '')
        self.declare_parameter('output_summary_path', 'run_summary.npy')

        bounds_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description='Competition boundary limits [xmin, xmax, ymin, ymax]'
        )
        self.declare_parameter(
            'bounds',
            [-1000.0, 1000.0, -1000.0, 1000.0],
            descriptor=bounds_descriptor
        )

        # Retrieve parameter values
        self.timeout_duration = self.get_parameter('timeout_duration_sec').value
        self.min_speed = self.get_parameter('min_speed_m_s').value
        self.max_speed = self.get_parameter('max_speed_m_s').value
        self.rover_radius = self.get_parameter('rover_radius_m').value
        self.auto_advance = self.get_parameter('auto_advance').value
        self.only_gnss_used = self.get_parameter('only_gnss_used').value
        mission_dir = self.get_parameter('mission_directory').value
        mission_name = self.get_parameter('mission_name').value
        yaml_params = self.get_parameter('yaml_params_file').value
        self.output_summary_path = self.get_parameter('output_summary_path').value
        custom_bounds = self.get_parameter('bounds').value

        # Load waypoints and starting pose
        waypoints, start_pose, optimal_segments = self._load_mission_definition(
            mission_dir, mission_name, yaml_params
        )

        # Initialize tracker
        self.tracker = WaypointTracker(
            waypoints=waypoints,
            start_pose=start_pose,
            rover_radius=self.rover_radius,
            auto_advance=self.auto_advance,
            optimal_path_segments=optimal_segments
        )

        # Calculate time bounds based on path length and speeds
        self.total_ideal_distance = self.tracker.total_ideal_distance
        self.t_max_computed = self.total_ideal_distance / max(0.01, self.min_speed)
        self.t_min_computed = self.total_ideal_distance / max(0.01, self.max_speed)
        self.minimum_time = self.t_min_computed
        self.maximum_time = min(self.timeout_duration, self.t_max_computed)

        self.get_logger().info(f"Loaded {len(waypoints)} Waypoints. Start Pose: {start_pose}")
        self.get_logger().info(f"Ideal Path Distance: {self.total_ideal_distance:.2f} m")
        self.get_logger().info(
            f"Speed Bounds: Min={self.min_speed} m/s, Max={self.max_speed} m/s | "
            f"Computed T_min (from max speed): {self.t_min_computed:.2f} s | "
            f"Computed T_max (from min speed): {self.t_max_computed:.2f} s | "
            f"Mission Max Time Limit: {self.timeout_duration:.2f} s (20 mins sim time)"
        )

        # Initialize Safety and Score engines
        self.safety_engine = CriticalRuleEnforcer(
            bounds=custom_bounds,
            timeout_secs=self.timeout_duration
        )

        score_config = ScoreCalculatorConfig(
            rover_radius=self.rover_radius,
            max_time_limit=self.timeout_duration
        )
        self.score_engine = ScoreCalculator(score_config)

        # State variables
        self.sim_start_time = None
        self.current_sim_time = 0.0
        self.last_clock_time = 0.0
        self.total_waiting_time = 0.0
        self.run_finished = False

        self.current_x = start_pose[0]
        self.current_y = start_pose[1]
        self.current_speed = 0.0

        # Publishers
        self.status_pub = self.create_publisher(String, '/referee/status', 10)
        self.score_pub = self.create_publisher(String, '/referee/score', 10)

        # Subscribers to live simulation
        # Use /odom (SLAM-fused odometry) to match erc_waypoint_follower.py's position source
        self.create_subscription(Odometry, '/odom', self._pose_callback, 10)
        self.create_subscription(Bool, '/rover/collision_flag', self._collision_callback, 10)
        self.create_subscription(Clock, '/clock', self._clock_callback, 10)
        self.create_subscription(String, '/referee/command', self._command_callback, 10)

        # Evaluation loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self._evaluation_loop)

        self.get_logger().info("RefereeCoreNode initialized and listening to live simulation topics.")

    def _load_mission_definition(self, mission_dir, mission_name, yaml_params):
        """Loads waypoints and start pose from YAML parameter file or NPY files."""
        if yaml_params and os.path.exists(yaml_params):
            try:
                start_point, wp_coords = load_waypoints_from_yaml(yaml_params)
                self.get_logger().info(f"Loaded mission from YAML file: {yaml_params}")
                waypoints = [Waypoint(id=f"W{i+1}", x=pt[0], y=pt[1]) for i, pt in enumerate(wp_coords)]
                return waypoints, start_point, []
            except Exception as e:
                self.get_logger().warn(f"Failed to load YAML params {yaml_params}: {e}")

        target_path = os.path.join(mission_dir, mission_name) if mission_dir else ""
        yaml_candidates = glob.glob(os.path.join(target_path, "*.yaml")) + glob.glob("*.yaml")
        if yaml_candidates:
            for y_file in yaml_candidates:
                try:
                    start_point, wp_coords = load_waypoints_from_yaml(y_file)
                    self.get_logger().info(f"Loaded mission from YAML parameter candidate: {y_file}")
                    waypoints = [Waypoint(id=f"W{i+1}", x=pt[0], y=pt[1]) for i, pt in enumerate(wp_coords)]
                    return waypoints, start_point, []
                except Exception:
                    continue

        wp_files = glob.glob(os.path.join(target_path, "wp*.npy")) + glob.glob(os.path.join(target_path, "*.npy"))
        wp_files = [f for f in wp_files if "optimal_path" not in f]
        ref_path_file = os.path.join(target_path, "optimal_path.npy")

        if wp_files and os.path.exists(wp_files[0]):
            raw_wps = np.load(wp_files[0], allow_pickle=True)
            waypoints = [Waypoint(id=f"W{i+1}", x=float(pt[0]), y=float(pt[1])) for i, pt in enumerate(raw_wps)]
            start_pose = (0.0, 0.0)
            optimal_segments = []
            if os.path.exists(ref_path_file):
                raw_paths = np.load(ref_path_file, allow_pickle=True)
                if len(raw_paths) > 0 and len(raw_paths[0]) > 0:
                    start_pose = (float(raw_paths[0][0][0]), float(raw_paths[0][0][1]))
                for seg in raw_paths:
                    optimal_segments.append([(float(p[0]), float(p[1])) for p in seg])
            return waypoints, start_pose, optimal_segments

        self.get_logger().warn("No mission file found. Using default waypoints [ (2,2), (-5,-5), (-15,5), (-10,8) ].")
        default_wps = [(2.0, 2.0), (-5.0, -5.0), (-15.0, 5.0), (-10.0, 8.0)]
        waypoints = [Waypoint(id=f"W{i+1}", x=pt[0], y=pt[1]) for i, pt in enumerate(default_wps)]
        return waypoints, (0.0, 0.0), []

    def _clock_callback(self, msg: Clock):
        t = msg.clock.sec + (msg.clock.nanosec * 1e-9)
        if self.sim_start_time is None:
            self.sim_start_time = t
            self.last_clock_time = t
            self.total_waiting_time = 0.0

        if self.tracker.state.name == "WAITING_FOR_COMMAND":
            self.total_waiting_time += (t - self.last_clock_time)

        self.current_sim_time = (t - self.sim_start_time) - self.total_waiting_time
        self.last_clock_time = t

    def _command_callback(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd == "NEXT":
            if self.tracker.resume_mission():
                self.get_logger().info("Referee Command: NEXT -> Target updated.")
        elif cmd in ("SKIP", "FAIL"):
            curr_pose = (self.current_x, self.current_y)
            if self.tracker.skip_current_waypoint(self.current_sim_time, curr_pose):
                self.get_logger().info(f"Referee Command: {cmd} -> Current waypoint marked as FAILED and skipped.")
        elif cmd.startswith("WAYPOINT_RESULT:") or cmd.startswith("RESULT:"):
            try:
                parts = cmd.split(":")
                if len(parts) == 5:
                    wp_id = parts[1]
                    rx = float(parts[2])
                    ry = float(parts[3])
                    dist_err = float(parts[4])
                else:
                    wp_id = "LAST"
                    rx = float(parts[1])
                    ry = float(parts[2])
                    dist_err = float(parts[3])
                if self.tracker.override_waypoint_result(wp_id, rx, ry, dist_err):
                    self.get_logger().info(f"Referee Command: RESULT -> Updated waypoint '{wp_id}' result to pose ({rx:.2f}, {ry:.2f}) | Error: {dist_err:.3f}m")
            except Exception as e:
                self.get_logger().warn(f"Failed to parse RESULT command {cmd}: {e}")
        elif cmd == "RETURN":
            self.tracker.record_return_request()
            self.get_logger().info(f"Referee Command: RETURN -> Return penalty registered ({self.tracker.num_returns} total).")
        elif cmd == "COLLISION":
            self.safety_engine.trigger_collision()

    def _pose_callback(self, msg: Odometry):
        if self.run_finished:
            return
        new_x = msg.pose.pose.position.x
        new_y = msg.pose.pose.position.y

        # Use the actual velocity from odometry twist (much more reliable than pose-delta)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.hypot(vx, vy)

        self.current_x = new_x
        self.current_y = new_y

        self.safety_engine.check_boundary(msg.pose.pose)
        self.tracker.update_position(
            (self.current_x, self.current_y),
            self.current_sim_time,
            self.current_speed
        )

    def _collision_callback(self, msg: Bool):
        if self.run_finished:
            return
        if msg.data:
            self.safety_engine.trigger_collision()

    def _save_summary_file(self, result_dict):
        """Helper to continuously persist run_summary.npy so it is never missing."""
        try:
            summary_array = np.array(result_dict, dtype=object)
            np.save(self.output_summary_path, summary_array)
            # Also save to fixed reports directory as a reliable fallback
            ws_root = os.environ.get('ROAR_WS', str(Path(__file__).resolve().parents[4]))
            fixed_reports_dir = os.path.join(ws_root, 'reports', 'referee')
            os.makedirs(fixed_reports_dir, exist_ok=True)
            np.save(os.path.join(fixed_reports_dir, 'run_summary.npy'), summary_array)
        except Exception as e:
            self.get_logger().warn(f"Failed to write summary file: {e}")

    def _evaluation_loop(self):
        if self.run_finished:
            return

        waypoints_remaining = len(self.tracker.waypoints) - self.tracker.waypoints_completed
        is_completed = (self.tracker.state.name == "COMPLETED")

        status, msg_log, is_fatal = self.safety_engine.evaluate_terminal_state(
            self.current_x,
            self.current_y,
            self.current_sim_time,
            waypoints_remaining,
            is_completed=is_completed
        )

        # Periodically compute current score and persist summary
        score_dict = self.score_engine.final_score(
            waypoint_details=self.tracker.waypoint_records,
            back_to_start_detail=self.tracker.back_to_start_record,
            num_return_requests=self.tracker.num_returns,
            only_gnss_used=self.only_gnss_used,
            elapsed_time=self.current_sim_time,
            is_fatal=is_fatal
        )
        score_dict["terminal_status"] = status
        score_dict["is_final"] = is_completed or is_fatal or (status in ["KILL", "TIMEOUT", "SUCCESS"])
        score_dict["t_min_computed_sec"] = float(self.t_min_computed)
        score_dict["t_max_computed_sec"] = float(self.t_max_computed)
        score_dict["minimum_time_sec"] = float(self.minimum_time)
        score_dict["maximum_time_sec"] = float(self.maximum_time)
        score_dict["ideal_distance_m"] = float(self.total_ideal_distance)
        score_dict["actual_distance_m"] = float(self.tracker.actual_distance)

        # Save summary continuously so file exists at all times
        self._save_summary_file(score_dict)

        if is_fatal or status in ["KILL", "TIMEOUT", "SUCCESS"] or is_completed:
            self._conclude_run(status=status, is_fatal=is_fatal)
            return

        score_msg = String()
        score_msg.data = json.dumps(score_dict)
        self.score_pub.publish(score_msg)

        status_msg = String()
        status_str = "WAITING FOR COMMAND" if self.tracker.state.name == "WAITING_FOR_COMMAND" else "RUNNING"
        status_msg.data = f"{status_str} | Waypoints: {self.tracker.waypoints_completed}/{len(self.tracker.waypoints)}"
        self.status_pub.publish(status_msg)

    def _conclude_run(self, status: str, is_fatal: bool):
        self.run_finished = True

        final_result = self.score_engine.final_score(
            waypoint_details=self.tracker.waypoint_records,
            back_to_start_detail=self.tracker.back_to_start_record,
            num_return_requests=self.tracker.num_returns,
            only_gnss_used=self.only_gnss_used,
            elapsed_time=self.current_sim_time,
            is_fatal=is_fatal
        )

        final_result["terminal_status"] = status
        final_result["is_final"] = True
        final_result["t_min_computed_sec"] = float(self.t_min_computed)
        final_result["t_max_computed_sec"] = float(self.t_max_computed)
        final_result["minimum_time_sec"] = float(self.minimum_time)
        final_result["maximum_time_sec"] = float(self.maximum_time)
        final_result["ideal_distance_m"] = float(self.total_ideal_distance)
        final_result["actual_distance_m"] = float(self.tracker.actual_distance)

        score_msg = String()
        score_msg.data = json.dumps(final_result)
        self.score_pub.publish(score_msg)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        self._save_summary_file(final_result)

        self.get_logger().info(
            f"Run Concluded: Status={status} | Final Score={final_result['final_score']:.2f}/300.0 | "
            f"NumPy summary saved to {self.output_summary_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RefereeCoreNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
