import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid

from .trajectory_error import TrajectoryErrorTracker
from .drift_rate import DriftRateCalculator
from .static_drift_monitor import StaticDriftMonitor
from .topography_validator import TopographyValidator
from .slam_scorer import SlamScorer
from .report_writer import ReportWriter

class SlamEvaluatorNode(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        
        self.declare_parameter('ground_truth_topic', '/zed2i/zed_node/pose_with_covariance')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('occupancy_topic', '/active_map/occupancy')
        ws_root = os.environ.get('ROAR_WS', str(Path(__file__).resolve().parents[5]))
        self.declare_parameter('output_dir', os.path.join(ws_root, 'reports', 'slam'))
        self.declare_parameter('truth_heightmap_path', '')
        
        # Modules
        self.traj_tracker = TrajectoryErrorTracker(use_3d=False) # 2D recommended
        self.drift_calc = DriftRateCalculator(window_size_sec=30.0)
        self.static_monitor = StaticDriftMonitor()
        self.topo_validator = TopographyValidator(self.get_parameter('truth_heightmap_path').value)
        self.scorer = SlamScorer()
        self.reporter = ReportWriter(self.get_parameter('output_dir').value)

        # Latest ground truth pose, updated independently of odom arrival.
        self.latest_gt_pose = None

        # Subscriptions
        self.cmd_sub = self.create_subscription(Twist, self.get_parameter('cmd_vel_topic').value, self.cmd_cb, 10)
        self.occ_sub = self.create_subscription(OccupancyGrid, self.get_parameter('occupancy_topic').value, self.occ_cb, 1)

        # Ground truth is the ZED pose topic (geometry_msgs/PoseWithCovarianceStamped);
        # the estimated pose is nav_msgs/Odometry.
        #
        # We deliberately do NOT pair them with an ApproximateTimeSynchronizer:
        # that requires both streams' header stamps to land within a tight
        # slop window, which silently yields zero synced samples (and a
        # perfect score) the moment either publisher's clock domain or
        # stamping differs from the other's -- with no error anywhere.
        # Instead we track the latest ground truth pose and score against it
        # whenever a new odom estimate arrives, same approach as the
        # navStack_2026 slam_evaluator.
        self.gt_sub = self.create_subscription(PoseWithCovarianceStamped, self.get_parameter('ground_truth_topic').value, self.gt_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.get_parameter('odom_topic').value, self.odom_cb, 50)

        self.create_timer(5.0, self._log_sample_count)

        self.get_logger().info("SLAM Evaluator Node Started. Waiting for data...")

    def _log_sample_count(self):
        self.get_logger().info(f"Trajectory samples recorded so far: {self.traj_tracker.count}")

    def cmd_cb(self, msg: Twist):
        t = self.get_clock().now().nanoseconds / 1e9
        self.static_monitor.update_cmd_vel(t, msg.linear.x, msg.angular.z)

    def occ_cb(self, msg: OccupancyGrid):
        # We only really need to evaluate the map at the end, but we keep the latest reference
        self.latest_occupancy = msg

    def gt_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.latest_gt_pose = (p.x, p.y, p.z)

    def odom_cb(self, msg: Odometry):
        if self.latest_gt_pose is None:
            return  # no ground truth received yet

        t = self.get_clock().now().nanoseconds / 1e9

        p_gt = self.latest_gt_pose
        p_est = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

        # Only log Trajectory Error at ~5Hz to save space/time
        # Since IESKF could run at 100Hz, we can decimate here by checking time delta
        if not hasattr(self, 'last_traj_log_time'):
            self.last_traj_log_time = t - 1.0

        if t - self.last_traj_log_time >= 0.2: # 5Hz
            self.traj_tracker.update(t, p_gt, p_est)
            self.last_traj_log_time = t

        # Update static monitor with pose every time to catch phantom drift accurately
        self.static_monitor.update_pose(t, p_est[0], p_est[1], p_est[2])

    def finalize(self):
        t = self.get_clock().now().nanoseconds / 1e9
        self.static_monitor.finalize(t)
        
        # Evaluate map if we have it
        if hasattr(self, 'latest_occupancy'):
            res = self.latest_occupancy.info.resolution
            ox = self.latest_occupancy.info.origin.position.x
            oy = self.latest_occupancy.info.origin.position.y
            w = self.latest_occupancy.info.width
            h = self.latest_occupancy.info.height
            grid = self.latest_occupancy.data
            
            # This would pass into topo_validator, omitted full integration here for brevity
            # metrics = self.topo_validator.evaluate_live_map(...)
        
        self.reporter.generate_full_report(
            self.traj_tracker, 
            self.drift_calc, 
            self.static_monitor, 
            self.topo_validator, 
            self.scorer
        )
        print("SLAM Evaluator finalized.")

def main(args=None):
    rclpy.init(args=args)
    node = SlamEvaluatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
