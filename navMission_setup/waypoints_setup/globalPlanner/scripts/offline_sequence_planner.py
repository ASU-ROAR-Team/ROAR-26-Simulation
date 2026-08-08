#!/usr/bin/env python3
"""Merged & Calibrated Offline Sequence Planner Node.

Loads terrain heightmaps (.npz / .png) or costmap CSVs matching the ERC Marsyard
coordinate frame (origin x=-4.55m, y=-10.34m, resolution=0.1m, size=419x274).
Publishes map layers with TRANSIENT_LOCAL QoS, teleports /odom between waypoints
(Start -> W1 -> W3 -> W2 -> S8 -> Start), requests D* Lite global plans, and exports
the 3D trajectory (x, y, z) to CSV.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import cv2
import numpy as np
import csv
import math
import time
import os
import re

class OfflineSequencePlanner(Node):
    def __init__(self):
        super().__init__('offline_sequence_planner')

        # Launch parameters: point directly at the two files that belong together
        self.declare_parameter('waypoints_file', 'waypoints.csv')
        self.declare_parameter('heightmap_file', 'heightmap_world.npz')

        _here = os.path.dirname(os.path.abspath(__file__))
        cwd = os.getcwd()

        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('dstar_navigation')
        except Exception:
            pkg_share = _here

        def resolve_file(filename):
            """Recursively search the whole workspace for filename."""
            if os.path.isabs(filename):
                if os.path.exists(filename):
                    return os.path.abspath(filename)
                raise FileNotFoundError(f"File not found: {filename}")

            workspace_root = os.path.abspath(
                os.path.join(_here, '..', '..', '..', '..')
            )

            for root, dirs, files in os.walk(workspace_root):
                if filename in files:
                    path = os.path.join(root, filename)
                    self.get_logger().info(f"Found '{filename}' at: {path}")
                    return os.path.abspath(path)

            raise FileNotFoundError(
                f"Could not find '{filename}' anywhere under {workspace_root}"
            )

        def parse_scenario_and_index(waypoints_filename):
            """'wp00_323.csv' -> ('00', '323')."""
            base = os.path.splitext(os.path.basename(waypoints_filename))[0]
            match = re.match(r'^wp(\d+)_(\d+)$', base)
            if not match:
                raise ValueError(
                    f"Waypoints filename '{waypoints_filename}' does not match "
                    f"expected pattern 'wp<scenario>_<index>.csv'"
                )
            return match.group(1), match.group(2)

        def parse_scenario(heightmap_filename):
            """'heightmap00.npz' -> '00'."""
            base = os.path.splitext(os.path.basename(heightmap_filename))[0]
            match = re.match(r'^heightmap(\d+)$', base)
            if not match:
                raise ValueError(
                    f"Heightmap filename '{heightmap_filename}' does not match "
                    f"expected pattern 'heightmap<scenario>.npz'"
                )
            return match.group(1)

        # Declarations of parameters
        self.declare_parameter('heightmap_npz', 'heightmap_world.npz')
        self.declare_parameter('heightmap_png', 'heightmap.png')
        self.declare_parameter('min_height', 0.0)      # meters
        self.declare_parameter('max_height', 1.5)      # meters
        self.declare_parameter('resolution', 0.1)      # meters/cell
        self.declare_parameter('origin_x', -4.552489)  # meters
        self.declare_parameter('origin_y', -10.339172) # meters
        self.declare_parameter('center_origin', False)
        self.declare_parameter('max_safe_slope_deg', 15.0)
        self.declare_parameter('max_safe_step_m', 0.30)

        # Read launch-supplied filenames
        waypoints_name = self.get_parameter('waypoints_file').value
        heightmap_name = self.get_parameter('heightmap_file').value

        # Resolve inputs anywhere under the workspace
        self.waypoints_file = resolve_file(waypoints_name)
        self.npz_file = resolve_file(heightmap_name)
        self.costmap_csv = resolve_file('costmap.csv')
        self.png_file = resolve_file('heightmap.png')

        # Derive scenario identity from the filenames themselves
        wp_scenario_id, self.waypoint_index = parse_scenario_and_index(waypoints_name)
        hm_scenario_id = parse_scenario(heightmap_name)

        if wp_scenario_id != hm_scenario_id:
            self.get_logger().warn(
                f"Scenario mismatch: waypoints file '{waypoints_name}' implies "
                f"scenario '{wp_scenario_id}' but heightmap file '{heightmap_name}' "
                f"implies scenario '{hm_scenario_id}'. Proceeding with the waypoints' "
                f"scenario id ('{wp_scenario_id}')."
            )

        self.scenario_id = wp_scenario_id
        self.combined_id = f"{self.scenario_id}_{self.waypoint_index}"

        # Output names — scenario- and waypoint-set-specific so runs never overwrite each other
        self.output_csv_data = os.path.normpath(os.path.join(_here, '..', '..', 'data', f'path{self.combined_id}.csv'))
        self.output_csv_cwd = os.path.normpath(os.path.join(cwd, f'path{self.combined_id}.csv'))
        self.output_costmap_csv_data = os.path.normpath(os.path.join(_here, '..', '..', 'data', f'costmap{self.scenario_id}.csv'))
        self.output_costmap_csv_cwd = os.path.normpath(os.path.join(cwd, f'costmap{self.scenario_id}.csv'))

        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.resolution = float(self.get_parameter('resolution').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.max_safe_slope = float(self.get_parameter('max_safe_slope_deg').value)
        self.max_safe_step = float(self.get_parameter('max_safe_step_m').value)

        # QoS for transient local map latching
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

        self.waypoints = []
        self.current_leg = 0
        self.latest_path = None
        self.all_generated_paths = []
        self.z_meters_map = None

        self.get_logger().info("Starting Calibrated MotionPlanning_dev Offline Sequence Planner...")

        if os.path.exists(self.npz_file):
            self.load_and_publish_from_npz()
        elif os.path.exists(self.costmap_csv):
            self.get_logger().warn(f"'{self.npz_file}' not found. Falling back to CSV costmap.")
            self.load_and_publish_from_csv()
        elif os.path.exists(self.png_file):
            self.get_logger().warn("Falling back to heightmap PNG.")
            self.load_and_publish_from_png()
        else:
            self.get_logger().error("No heightmap NPZ, costmap CSV, or PNG found!")

        self.load_waypoints()

        self.timer_started = False
        self.create_timer(2.5, self.start_sequence)

    def load_and_publish_from_npz(self):
        try:
            self.get_logger().info(f"Loading terrain heightmap NPZ: {self.npz_file}")
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

            self.get_logger().info(
                f"Loaded NPZ Grid ({nx}x{ny}, Resolution: {self.resolution:.4f}m/cell, Origin: ({self.origin_x:.2f}m, {self.origin_y:.2f}m))"
            )

            # Calculate terrain slope and step costmap
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

            # Publish /map
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

            # Publish /active_map/heightmap
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

            self.export_costmap_to_csv(costmap)
            self.get_logger().info(f"Successfully published map layers from NPZ ({nx}x{ny}).")
        except Exception as e:
            self.get_logger().error(f"Failed to load heightmap NPZ: {e}")

    def export_costmap_to_csv(self, grid_2d):
        height, width = grid_2d.shape
        header = [''] + [str(i) for i in range(width)]
        _here = os.path.dirname(os.path.abspath(__file__))
        cwd = os.getcwd()
        targets = set([
            os.path.normpath(os.path.join(_here, '..', '..', 'data', f'costmap{self.scenario_id}.csv')),
            os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'data', f'costmap{self.scenario_id}.csv')),
            os.path.normpath(os.path.join(cwd, 'data', f'costmap{self.scenario_id}.csv')),
            os.path.normpath(os.path.join(cwd, f'costmap{self.scenario_id}.csv')),
        ])
        for target_path in targets:
            try:
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with open(target_path, 'w', newline='') as f:
                    f.write(f"# resolution={self.resolution}\n")
                    f.write(f"# origin_x={self.origin_x} origin_y={self.origin_y}\n")
                    writer = csv.writer(f)
                    writer.writerow(header)
                    for y in range(height):
                        row = [str(y)] + [str(int(grid_2d[y, x])) for x in range(width)]
                        writer.writerow(row)
                self.get_logger().info(f"Exported costmap CSV ({width}x{height}, origin={self.origin_x:.2f},{self.origin_y:.2f}) to: {target_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to export costmap CSV to {target_path}: {e}")

    def load_and_publish_from_csv(self):
        try:
            origin_x = self.origin_x
            origin_y = self.origin_y
            resolution = self.resolution
            lines = []
            with open(self.costmap_csv, 'r') as f:
                for line in f:
                    if line.startswith('#'):
                        if 'resolution=' in line:
                            resolution = float(line.split('resolution=')[1].split()[0])
                        if 'origin_x=' in line:
                            parts = line.split()
                            for p in parts:
                                if p.startswith('origin_x='):
                                    origin_x = float(p.split('=')[1])
                                elif p.startswith('origin_y='):
                                    origin_y = float(p.split('=')[1])
                    else:
                        lines.append(line)

            reader = csv.reader(lines)
            header = next(reader, None)
            if header is None:
                raise ValueError("Empty costmap CSV")
            width = len([h for h in header[1:] if h.strip() != ''])

            grid_data = []
            height = 0
            for row in reader:
                clean_row = [int(v.strip()) if v.strip() else 0 for v in row[1:width + 1]]
                if len(clean_row) < width:
                    clean_row.extend([0] * (width - len(clean_row)))
                grid_data.extend(clean_row)
                height += 1

            self.grid_width = width
            self.grid_height = height
            self.resolution = resolution
            self.origin_x = origin_x
            self.origin_y = origin_y

            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.info.resolution = resolution
            msg.info.width = width
            msg.info.height = height
            msg.info.origin.position.x = origin_x
            msg.info.origin.position.y = origin_y
            msg.data = grid_data
            self.map_pub.publish(msg)
            self.get_logger().info(f"Published Map from CSV ({width}x{height}, origin=({origin_x:.2f}, {origin_y:.2f})).")
        except Exception as e:
            self.get_logger().error(f"Failed to load costmap CSV: {e}")

    def load_and_publish_from_png(self):
        try:
            img = cv2.imread(self.png_file, cv2.IMREAD_UNCHANGED)
            if img is None:
                raise ValueError("Could not read image file.")

            if len(img.shape) == 3 and img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2GRAY)
            elif len(img.shape) == 3:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            img = cv2.transpose(img)
            img = cv2.flip(img, 0)
            height, width = img.shape
            self.grid_width = width
            self.grid_height = height
            max_val = 65535.0 if img.dtype == np.uint16 else 255.0

            normalized_img = img.astype(np.float32) / max_val
            self.z_meters_map = self.min_height + normalized_img * (self.max_height - self.min_height)

            sobel_x = cv2.Sobel(self.z_meters_map, cv2.CV_32F, 1, 0, ksize=3) / (8.0 * self.resolution)
            sobel_y = cv2.Sobel(self.z_meters_map, cv2.CV_32F, 0, 1, ksize=3) / (8.0 * self.resolution)
            gradient_mag = np.sqrt(sobel_x**2 + sobel_y**2)
            slope_deg = np.degrees(np.arctan(gradient_mag))
            laplacian = np.abs(cv2.Laplacian(self.z_meters_map, cv2.CV_32F))

            steepness_cost = np.clip((slope_deg / self.max_safe_slope) * 50.0, 0, 50)
            roughness_cost = np.clip((laplacian / 0.20) * 30.0, 0, 30)
            clean_map = np.clip(steepness_cost + roughness_cost, 0, 99).astype(np.int8)

            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = 'map'
            map_msg.info.resolution = self.resolution
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = self.origin_x
            map_msg.info.origin.position.y = self.origin_y
            map_msg.data = clean_map.flatten().tolist()
            self.map_pub.publish(map_msg)

            self.export_costmap_to_csv(clean_map)
        except Exception as e:
            self.get_logger().error(f"Failed to load heightmap PNG: {e}")


    def load_waypoints(self):
        try:
            raw_wps = []
            with open(self.waypoints_file, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) < 2:
                        continue
                    try:
                        raw_wps.append((float(row[0]), float(row[1])))
                    except ValueError:
                        continue

            self.waypoints = raw_wps
            # Append start waypoint (0.0, 0.0) at the end to close the mission loop
            if self.waypoints:
                self.waypoints.append(self.waypoints[0])
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from {self.waypoints_file} (including return to start).")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints CSV: {e}")

    def start_sequence(self):
        if self.timer_started:
            return
        self.timer_started = True
        self.process_next_leg()

    def process_next_leg(self):
        if self.current_leg >= len(self.waypoints) - 1:
            self.get_logger().info("SEQUENCE COMPLETE! All paths generated.")
            self.save_paths_to_file()
            rclpy.shutdown()
            return

        start_pt = self.waypoints[self.current_leg]
        end_pt = self.waypoints[self.current_leg + 1]

        self.get_logger().info(f"--- Computing Leg {self.current_leg + 1}: {start_pt} -> {end_pt} ---")
        self.teleport_robot(start_pt[0], start_pt[1])
        self.latest_path = None
        self.send_goal(end_pt[0], end_pt[1])

    def teleport_robot(self, x, y):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        time.sleep(0.5)

    def send_goal(self, x, y):
        self.action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info("Sending goal to D*...")
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by D* planner!")
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted. Waiting for /global_plan...")
        self.poll_timer = self.create_timer(0.1, self.check_for_path)

    def path_callback(self, msg):
        if self.latest_path is None and msg.poses:
            if self.current_leg < len(self.waypoints) - 1:
                end_pt = self.waypoints[self.current_leg + 1]
                last_pose = msg.poses[-1].pose.position
                dist = math.hypot(last_pose.x - end_pt[0], last_pose.y - end_pt[1])
                if dist < 1.5:  # 1.5m tolerance to target goal
                    self.latest_path = msg
                else:
                    self.get_logger().info(
                        f"Ignoring stale path ending at ({last_pose.x:.2f}, {last_pose.y:.2f}) "
                        f"far from goal {end_pt} (dist: {dist:.2f}m)."
                    )
            else:
                self.latest_path = msg

    def check_for_path(self):
        if self.latest_path is not None:
            self.poll_timer.cancel()
            self.all_generated_paths.append(self.latest_path)
            self.get_logger().info(f"Path received! Length: {len(self.latest_path.poses)} poses.")

            self.current_goal_handle.cancel_goal_async()
            self.current_leg += 1
            self.process_next_leg()

    def get_elevation_z(self, x, y):
        if self.z_meters_map is None:
            return 0.0
        px = int((x - self.origin_x) / self.resolution)
        py = int((y - self.origin_y) / self.resolution)
        if 0 <= px < self.grid_width and 0 <= py < self.grid_height:
            return float(self.z_meters_map[py, px])
        return 0.0

    def save_paths_to_file(self):
        _here = os.path.dirname(os.path.abspath(__file__))
        cwd = os.getcwd()
        targets = set([
            os.path.normpath(os.path.join(_here, '..', '..', 'data', f'path{self.combined_id}.csv')),
            os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'data', f'path{self.combined_id}.csv')),
            os.path.normpath(os.path.join(cwd, 'data', f'path{self.combined_id}.csv')),
            os.path.normpath(os.path.join(cwd, f'path{self.combined_id}.csv')),
        ])
        for target_path in targets:
            try:
                os.makedirs(os.path.dirname(target_path), exist_ok=True)
                with open(target_path, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['x', 'y', 'z'])
                    for path in self.all_generated_paths:
                        for pose_stamped in path.poses:
                            px = pose_stamped.pose.position.x
                            py = pose_stamped.pose.position.y
                            pz = self.get_elevation_z(px, py)
                            writer.writerow([px, py, pz])
                self.get_logger().info(f"Successfully saved 3D path data (x, y, z) to: {target_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to write output CSV to {target_path}: {e}")

        # Automatically generate matching Path Attraction Costmap Layer
        try:
        # pyrefly: ignore [missing-import]
            from offline_tools.scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
        except ImportError:
            try:
                # pyrefly: ignore [missing-import]
                from scripts.mapping.generate_path_attraction_costmap import generate_attraction_costmap, export_attraction_costmap
            except ImportError:
                generate_attraction_costmap = None

        if generate_attraction_costmap is not None:
            all_pts = []
            for path in self.all_generated_paths:
                for ps in path.poses:
                    all_pts.append((ps.pose.position.x, ps.pose.position.y))
            meta = {'resolution': self.resolution, 'origin_x': self.origin_x, 'origin_y': self.origin_y,
                    'width': self.grid_width, 'height': self.grid_height}
            grid = generate_attraction_costmap(all_pts, meta, max_bonus=30.0, corridor_radius=1.5, sigma=0.5)

            attr_targets = set([
                os.path.normpath(os.path.join(_here, '..', '..', 'data', f'path{self.combined_id}_attraction_costmap.csv')),
                os.path.normpath(os.path.join(cwd, 'path-planning', 'globalPlanner', 'offline_tools', 'data', f'path{self.combined_id}_attraction_costmap.csv')),
                os.path.normpath(os.path.join(cwd, 'data', f'path{self.combined_id}_attraction_costmap.csv')),
            ])
            for attr_path in attr_targets:
                try:
                    export_attraction_costmap(grid, meta, attr_path)
                    self.get_logger().info(f"Generated attraction costmap layer: {attr_path}")
                except Exception as e:
                    self.get_logger().error(f"Failed to write attraction costmap: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = OfflineSequencePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()