#!/usr/bin/env python3
import os
import sys
import math
import argparse
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus

class ERCWaypointFollower(Node):
    def __init__(self):
        super().__init__('erc_waypoint_follower')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def navigate_to_waypoint(self, x, y):
        self.get_logger().info(f"\n---> Connecting to '/navigate_to_pose' Action Server...")
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server '/navigate_to_pose' not available.")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending waypoint goal: ({x:.2f}, {y:.2f})")
        send_goal_future = self.client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the action server.")
            return False

        self.get_logger().info("Goal accepted. Moving to waypoint...")
        get_result_future = goal_handle.get_result_async()
        
        # Wait for execution result
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Target waypoint reached!")
            if self.current_pose is not None:
                rx = self.current_pose.position.x
                ry = self.current_pose.position.y
                dist = math.hypot(rx - x, ry - y)
                self.get_logger().info(f"Final Pose: ({rx:.2f}, {ry:.2f}) | Distance error: {dist:.3f}m")
            return True
        else:
            self.get_logger().error(f"Failed to reach waypoint. Goal status: {result.status}")
            return False

def load_waypoints_from_yaml(file_path):
    try:
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)
        
        start_end = None
        waypoints_raw = None

        # Recursively search for start_end and waypoints under any ros__parameters
        def search_dict(d):
            nonlocal start_end, waypoints_raw
            if not isinstance(d, dict):
                return
            if 'ros__parameters' in d:
                params = d['ros__parameters']
                # Search under sub-plugin dictionaries like 'GridBased'
                for k, v in params.items():
                    if isinstance(v, dict):
                        if 'start_end' in v:
                            start_end = v['start_end']
                        if 'waypoints' in v:
                            waypoints_raw = v['waypoints']
                
                # Check top-level parameter keys
                if 'start_end' in params:
                    start_end = params['start_end']
                if 'waypoints' in params:
                    waypoints_raw = params['waypoints']
                    
            for val in d.values():
                if isinstance(val, dict):
                    search_dict(val)

        search_dict(data)
        
        if start_end is None or waypoints_raw is None:
            print(f"Error: Could not find 'start_end' and 'waypoints' parameters in {file_path}")
            sys.exit(1)
            
        if len(waypoints_raw) % 2 != 0:
            print("Error: 'waypoints' parameter must have an even number of coordinates.")
            sys.exit(1)
            
        waypoints = []
        for i in range(0, len(waypoints_raw), 2):
            waypoints.append((float(waypoints_raw[i]), float(waypoints_raw[i+1])))
            
        start_point = (float(start_end[0]), float(start_end[1]))
        return start_point, waypoints
    except Exception as e:
        print(f"Error reading or parsing YAML file {file_path}: {e}")
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description="ERC Waypoint Navigation with YAML parameters")
    parser.add_argument('--params', type=str, required=True, 
                        help="Path to YAML parameter file (e.g. dstar_params.yaml or roar_nav2_sim.yaml)")
    args = parser.parse_known_args()[0]

    if not os.path.exists(args.params):
        print(f"Params file not found: {args.params}")
        sys.exit(1)

    start_point, waypoints = load_waypoints_from_yaml(args.params)
    print(f"Loaded Start/End point: {start_point}")
    print(f"Loaded Waypoints sequence: {waypoints}")

    # Build final sequence: visit w1 -> w2 -> w3 -> w4 -> return to start_point
    route = waypoints + [start_point]

    rclpy.init()
    node = ERCWaypointFollower()

    try:
        for idx, (x, y) in enumerate(route):
            is_return = (idx == len(route) - 1)
            wp_label = "Return to Start" if is_return else f"Waypoint {idx + 1}"
            
            print(f"\n==================================================")
            print(f" {wp_label}: Target ({x:.2f}, {y:.2f})")
            print(f"==================================================")
            
            success = node.navigate_to_waypoint(x, y)
            
            if success:
                print(f"\n[OK] Rover stopped at {wp_label}.")
                if is_return:
                    print("Returned to start safely.")
                else:
                    input(">>> Press [Enter] once the judge approves moving to the next waypoint...")
            else:
                print(f"\n[ERROR] Action failed for {wp_label}.")
                choice = input(">>> Type 'skip' to skip to next waypoint, or press [Enter] to retry: ")
                if choice.strip().lower() == 'skip':
                    continue
                else:
                    route.insert(idx + 1, (x, y))
                    
        print("\nAll waypoints and return sequence completed successfully!")
        
    except KeyboardInterrupt:
        print("\nNavigation interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
