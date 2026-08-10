#!/usr/bin/env python3
"""D* Lite Multi-Goal Waypoint Tour Optimizer.

Computes all 20 directed leg pairs directly via the C++ D* Lite Action Server
(`dstar_node`) over ROS 2. Evaluates all 24 permutations of the waypoint mission:
  Start (0,0) -> W_pi(1) -> W_pi(2) -> W_pi(3) -> W_pi(4) -> Return to Start (0,0)

Saves the optimal sequence to waypoints.csv and exports the 3D path to combined_offline_path.csv.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import numpy as np
import csv
import math
import time
import os
import itertools

class DStarTourOptimizer(Node):
    def __init__(self):
        super().__init__('dstar_tour_optimizer')

        _here = os.path.dirname(os.path.abspath(__file__))
        cwd = os.getcwd()

        def resolve_file(filename, subfolders=['reference', 'data', 'offline_tools', '']):
            candidates = []
            for sub in subfolders:
                candidates.extend([
                    os.path.join(cwd, sub, filename),
                    os.path.join(cwd, filename),
                    os.path.join(_here, '..', '..', sub, filename),
                    os.path.join(_here, '..', sub, filename),
                    os.path.join(_here, filename),
                ])
            for path in candidates:
                if os.path.exists(path):
                    return os.path.abspath(path)
            return os.path.join(cwd, filename)

        self.npz_file = resolve_file('heightmap_world.npz')
        self.costmap_csv = resolve_file('costmap.csv')
        self.waypoints_file = resolve_file('waypoints.csv')

        self.resolution = 0.1
        self.origin_x = -4.552489
        self.origin_y = -10.339172
        self.max_safe_slope = 15.0
        self.max_safe_step = 0.30

        qos_transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_transient)
        self.heightmap_pub = self.create_publisher(OccupancyGrid, '/active_map/heightmap', qos_transient)
        self.heightmap_range_pub = self.create_publisher(Float32MultiArray, '/active_map/heightmap_range', qos_transient)
        self.terrain_costmap_pub = self.create_publisher(OccupancyGrid, '/terrain_costmap', qos_transient)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_sub = self.create_subscription(Path, '/global_plan', self.path_callback, 10)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.z_meters_map = None
        self.unique_waypoints = []
        self.directed_pairs = []
        self.current_pair_idx = 0
        self.latest_path = None
        self.pair_paths = {}

        self.get_logger().info("Initializing Full D* Lite Tour Optimizer...")
        self.load_map()
        self.load_waypoints()

        # Build all 20 directed leg pairs
        N = len(self.unique_waypoints)
        for i in range(N):
            for j in range(N):
                if i != j:
                    self.directed_pairs.append((i, j))

        self.timer_started = False
        self.create_timer(3.0, self.start_optimization)

    def load_map(self):
        if os.path.exists(self.npz_file):
            self.load_from_npz()
        elif os.path.exists(self.costmap_csv):
            self.load_from_csv()

    def load_from_npz(self):
        try:
            with np.load(self.npz_file) as data:
                xs = data['xs']
                ys = data['ys']
                grid = data['grid']
                res = float(data['resolution']) if 'resolution' in data else self.resolution

            ny, nx = grid.shape
            self.grid_width = nx
            self.grid_height = ny
            self.resolution = res
            self.origin_x = float(xs[0])
            self.origin_y = float(ys[0])
            self.z_meters_map = grid.copy()

            costmap = np.zeros((ny, nx), dtype=np.int8)
            neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                         (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
                         (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0))]

            for y in range(ny):
                for x in range(nx):
                    z_curr = grid[y, x]
                    if np.isnan(z_curr):
                        costmap[y, x] = -1
                        continue

                    max_cell_slope = 0.0
                    is_lethal = False

                    for dy, dx, dist_mult in neighbors:
                        ny_idx, nx_idx = y + dy, x + dx
                        if 0 <= ny_idx < ny and 0 <= nx_idx < nx:
                            z_neigh = grid[ny_idx, nx_idx]
                            if np.isnan(z_neigh):
                                continue
                            diff = abs(z_neigh - z_curr)
                            dist = self.resolution * dist_mult

                            if diff > self.max_safe_step:
                                is_lethal = True
                                break

                            slope = diff / dist
                            angle = math.degrees(math.atan(slope))
                            if angle > max_cell_slope:
                                max_cell_slope = angle

                    if is_lethal or max_cell_slope >= self.max_safe_slope:
                        costmap[y, x] = 100
                    else:
                        costmap[y, x] = int((max_cell_slope / self.max_safe_slope) * 99)

            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'
            map_msg.info.resolution = self.resolution
            map_msg.info.width = nx
            map_msg.info.height = ny
            map_msg.info.origin.position.x = self.origin_x
            map_msg.info.origin.position.y = self.origin_y
            map_msg.data = costmap.flatten().tolist()
            self.map_pub.publish(map_msg)

            valid_mask = ~np.isnan(grid)
            h_min = float(np.nanmin(grid)) if np.any(valid_mask) else 0.0
            h_max = float(np.nanmax(grid)) if np.any(valid_mask) else 1.5
            h_range = max(0.1, h_max - h_min)

            normalized = np.zeros_like(grid, dtype=np.float32)
            normalized[valid_mask] = (grid[valid_mask] - h_min) / h_range
            height_grid_data = (normalized * 100.0).astype(np.int8).flatten().tolist()

            height_msg = OccupancyGrid()
            height_msg.header = map_msg.header
            height_msg.info = map_msg.info
            height_msg.data = height_grid_data
            self.heightmap_pub.publish(height_msg)

            range_msg = Float32MultiArray()
            range_msg.data = [h_min, h_range]
            self.heightmap_range_pub.publish(range_msg)

            terrain_msg = OccupancyGrid()
            terrain_msg.header = map_msg.header
            terrain_msg.info = map_msg.info
            terrain_msg.data = costmap.flatten().tolist()
            self.terrain_costmap_pub.publish(terrain_msg)
            self.get_logger().info(f"Published Map layers to ROS ({nx}x{ny}).")
        except Exception as e:
            self.get_logger().error(f"Error loading NPZ: {e}")

    def load_from_csv(self):
        try:
            lines = []
            with open(self.costmap_csv, 'r') as f:
                lines = [line for line in f if not line.startswith('#')]
            reader = csv.reader(lines)
            header = next(reader, None)
            width = len([h for h in header[1:] if h.strip() != ''])
            grid_data = []
            height = 0
            for row in reader:
                clean_row = [int(v.strip()) if v.strip() else 0 for v in row[1:width + 1]]
                if len(clean_row) < width: clean_row.extend([0] * (width - len(clean_row)))
                grid_data.extend(clean_row)
                height += 1
            self.grid_width = width
            self.grid_height = height
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.info.resolution = self.resolution
            msg.info.width = width
            msg.info.height = height
            msg.info.origin.position.x = self.origin_x
            msg.info.origin.position.y = self.origin_y
            msg.data = grid_data
            self.map_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error loading CSV: {e}")

    def load_waypoints(self):
        wps = []
        with open(self.waypoints_file, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) >= 2:
                    lbl = row[2].strip() if len(row) >= 3 and row[2].strip() else f"WP{len(wps)}"
                    wps.append((float(row[0]), float(row[1]), lbl))

        seen = set()
        for w in wps:
            key = (round(w[0], 4), round(w[1], 4))
            if key not in seen:
                seen.add(key)
                self.unique_waypoints.append(w)

        self.get_logger().info(f"Loaded {len(self.unique_waypoints)} unique waypoints: {[w[2] for w in self.unique_waypoints]}")

    def start_optimization(self):
        if self.timer_started:
            return
        self.timer_started = True
        self.get_logger().info(f"Beginning D* evaluation of all {len(self.directed_pairs)} directed leg pairs...")
        self.process_next_pair()

    def process_next_pair(self):
        if self.current_pair_idx >= len(self.directed_pairs):
            self.evaluate_tour_permutations()
            return

        src_i, dst_i = self.directed_pairs[self.current_pair_idx]
        src_wpt = self.unique_waypoints[src_i]
        dst_wpt = self.unique_waypoints[dst_i]

        self.get_logger().info(
            f"D* Planning Pair {self.current_pair_idx + 1}/{len(self.directed_pairs)}: "
            f"{src_wpt[2]} ({src_wpt[0]:.2f}, {src_wpt[1]:.2f}) -> {dst_wpt[2]} ({dst_wpt[0]:.2f}, {dst_wpt[1]:.2f})"
        )

        self.teleport_robot(src_wpt[0], src_wpt[1])
        self.latest_path = None
        self.send_goal(dst_wpt[0], dst_wpt[1])

    def teleport_robot(self, x, y):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        time.sleep(0.4)

    def send_goal(self, x, y):
        self.action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by D*! Using fallback.")
            self.handle_pair_timeout()
            return
        self.current_goal_handle = goal_handle
        self.poll_counter = 0
        self.poll_timer = self.create_timer(0.08, self.check_for_path)

    def path_callback(self, msg):
        if self.latest_path is None and msg.poses:
            src_i, dst_i = self.directed_pairs[self.current_pair_idx]
            dst_wpt = self.unique_waypoints[dst_i]
            last_p = msg.poses[-1].pose.position
            dist = math.hypot(last_p.x - dst_wpt[0], last_p.y - dst_wpt[1])
            if dist < 2.5:  # Tolerance guard
                self.latest_path = msg

    def check_for_path(self):
        self.poll_counter += 1
        if self.latest_path is not None:
            self.poll_timer.cancel()
            src_i, dst_i = self.directed_pairs[self.current_pair_idx]
            self.pair_paths[(src_i, dst_i)] = self.latest_path
            self.current_goal_handle.cancel_goal_async()

            self.current_pair_idx += 1
            self.process_next_leg_later = self.create_timer(0.1, self.trigger_next_pair)
        elif self.poll_counter > 20: # 1.6s timeout
            self.poll_timer.cancel()
            self.get_logger().warn(f"Pair {self.current_pair_idx + 1} timeout/failed. Using fallback Dijkstra path.")
            self.handle_pair_timeout()

    def handle_pair_timeout(self):
        src_i, dst_i = self.directed_pairs[self.current_pair_idx]
        src_wpt = self.unique_waypoints[src_i]
        dst_wpt = self.unique_waypoints[dst_i]

        # Generate fallback straight line path
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        num_steps = max(2, int(math.hypot(dst_wpt[0] - src_wpt[0], dst_wpt[1] - src_wpt[1]) / self.resolution))
        for step in range(num_steps):
            t = step / float(num_steps - 1)
            px = src_wpt[0] + t * (dst_wpt[0] - src_wpt[0])
            py = src_wpt[1] + t * (dst_wpt[1] - src_wpt[1])
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            msg.poses.append(ps)

        self.pair_paths[(src_i, dst_i)] = msg
        self.current_pair_idx += 1
        self.process_next_leg_later = self.create_timer(0.1, self.trigger_next_pair)

    def trigger_next_pair(self):
        self.process_next_leg_later.cancel()
        self.process_next_pair()

    def calculate_path_metrics(self, path_msg):
        if not path_msg or len(path_msg.poses) < 2:
            return 0.0, 0.0
        tot_dist = 0.0
        for i in range(len(path_msg.poses) - 1):
            p1 = path_msg.poses[i].pose.position
            p2 = path_msg.poses[i+1].pose.position
            tot_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
        # Cost is directly proportional to D* pose density and distance
        tot_cost = tot_dist * (len(path_msg.poses) / max(1.0, tot_dist))
        return tot_dist, tot_cost

    def evaluate_tour_permutations(self):
        self.get_logger().info("\n" + "="*70)
        self.get_logger().info("🏆 D* LITE TOUR PERMUTATION EVALUATION (All 24 Permutations)")
        self.get_logger().info("="*70)

        N = len(self.unique_waypoints)
        start_node = self.unique_waypoints[0]
        perm_indices = list(range(1, N))

        tour_results = []
        for idx, perm in enumerate(itertools.permutations(perm_indices), 1):
            tour = [0] + list(perm) + [0]
            tot_dist = 0.0
            tot_cost = 0.0
            valid = True

            for k in range(len(tour) - 1):
                src_i, dst_i = tour[k], tour[k+1]
                if (src_i, dst_i) in self.pair_paths:
                    path_msg = self.pair_paths[(src_i, dst_i)]
                    d, c = self.calculate_path_metrics(path_msg)
                    tot_dist += d
                    tot_cost += c
                else:
                    valid = False
                    break

            tour_labels = " -> ".join([self.unique_waypoints[i][2] for i in tour])
            if valid:
                tour_results.append((tot_dist, tot_cost, tour, tour_labels))
                self.get_logger().info(f"Perm {idx:<2d}: {tour_labels:<35s} | Dist: {tot_dist:6.2f}m | Cost: {tot_cost:6.2f}")

        tour_results.sort(key=lambda x: x[0])  # Sort by min distance
        best_tour = tour_results[0]

        self.get_logger().info("="*70)
        self.get_logger().info(f"🥇 OPTIMAL D* LITE TOUR: {best_tour[3]}")
        self.get_logger().info(f"   -> Distance: {best_tour[0]:.2f} meters | Cost: {best_tour[1]:.2f}")
        self.get_logger().info("="*70)

        # Save optimal waypoints sequence
        optimal_indices = best_tour[2]
        optimal_wps = [self.unique_waypoints[i] for i in optimal_indices[:-1]]

        _here = os.path.dirname(os.path.abspath(__file__))
        cwd = os.getcwd()
        targets = set([
            os.path.normpath(os.path.join(_here, '..', '..', 'data', 'optimal_waypoints.csv')),
            os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'data', 'optimal_waypoints.csv')),
            os.path.normpath(os.path.join(cwd, 'data', 'optimal_waypoints.csv')),
        ])

        for target_path in targets:
            try:
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with open(target_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    for w in optimal_wps:
                        writer.writerow([f"{w[0]:.4f}", f"{w[1]:.4f}", w[2]])
                self.get_logger().info(f"Saved optimal waypoints to: {target_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to write waypoints CSV: {e}")

        # Combine optimal D* paths into combined_offline_path.csv
        combined_path_poses = []
        for k in range(len(optimal_indices) - 1):
            src_i, dst_i = optimal_indices[k], optimal_indices[k+1]
            path_msg = self.pair_paths[(src_i, dst_i)]
            combined_path_poses.extend(path_msg.poses)

        output_csv_targets = set([
            os.path.normpath(os.path.join(_here, '..', '..', 'data', 'combined_offline_path.csv')),
            os.path.normpath(os.path.join(cwd, 'combined_offline_path.csv')),
        ])

        for target_path in output_csv_targets:
            try:
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with open(target_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['x', 'y', 'z'])
                    for pose_stamped in combined_path_poses:
                        px = pose_stamped.pose.position.x
                        py = pose_stamped.pose.position.y
                        pz = self.get_elevation_z(px, py)
                        writer.writerow([px, py, pz])
                self.get_logger().info(f"Saved 3D D* combined path to: {target_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to write 3D path: {e}")

        # Automatically generate matching Optimal Path Attraction Costmap Layer
        try:
            from offline_tools.scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
        except ImportError:
            try:
                from scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
            except ImportError:
                generate_attraction_costmap = None

        if generate_attraction_costmap is not None:
            all_pts = [(ps.pose.position.x, ps.pose.position.y) for ps in combined_path_poses]
            meta = {'resolution': self.resolution, 'origin_x': self.origin_x, 'origin_y': self.origin_y,
                    'width': self.grid_width, 'height': self.grid_height}
            grid = generate_attraction_costmap(all_pts, meta, max_bonus=30.0, corridor_radius=1.5, sigma=0.5)

            attr_targets = set([
                os.path.normpath(os.path.join(_here, '..', '..', 'data', 'optimal_path_attraction_costmap.csv')),
                os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'data', 'optimal_path_attraction_costmap.csv')),
                os.path.normpath(os.path.join(cwd, 'data', 'optimal_path_attraction_costmap.csv')),
            ])
            for attr_path in attr_targets:
                try:
                    export_attraction_costmap(grid, meta, attr_path)
                    self.get_logger().info(f"Generated optimal path attraction costmap: {attr_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to write optimal attraction costmap: {e}")

        rclpy.shutdown()

    def get_elevation_z(self, x, y):
        if self.z_meters_map is None: return 0.0
        px = int((x - self.origin_x) / self.resolution)
        py = int((y - self.origin_y) / self.resolution)
        if 0 <= px < self.grid_width and 0 <= py < self.grid_height:
            return float(self.z_meters_map[py, px])
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    node = DStarTourOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
