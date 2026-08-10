"""Full-stack bringup: D* Lite global planner + MPPI local controller.

This launch file composes the global planner module and the MPPI local
controller module into a single, integrated navigation stack.

Data flow
---------
  dynamic_map_publisher ──► dstar_global_planner ──► /global_plan
  path_simulator        ──► /robot_pose (pose feedback to planner)
  /global_plan          ──► controller_server (MPPI) ──► /cmd_vel

Nodes / processes launched
--------------------------
Global planner side
  - dstar_global_planner   : D* Lite planner (nav_msgs/Path on /global_plan)
  - dynamic_map_publisher  : Occupancy grid for the chosen map scenario
  - path_simulator         : Advances robot pose at 0.5 m/s along /global_plan
  - map_to_odom_tf         : Static transform  map → odom  (identity)

Local controller side
  - fake_robot             : Python process that simulates robot state
  - map_server             : Serves the empty occupancy-grid map (Nav2)
  - controller_server      : MPPI local controller (Nav2)
  - planner_server         : Nav2 planner server
  - lifecycle_manager      : Autostart manager for all Nav2 nodes

Shared
  - rviz2                  : Single RViz2 instance with dstar_test.rviz

Launch arguments
----------------
map_scenario : Map scenario for dynamic_map_publisher (default: 'open').
start_x      : Robot start X coordinate in metres (default: 0.5).
start_y      : Robot start Y coordinate in metres (default: 0.5).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# ── MPPI / empty_test absolute paths (mirror test_all.launch.py) ──────────────
MPPI_BASE   = '/home/amrtamer/nav-stack_2026/empty_test'
MPPI_PARAMS = f'{MPPI_BASE}/params/mppi_empty.yaml'
MPPI_MAP    = f'{MPPI_BASE}/maps/empty_map.yaml'
FAKE_ROBOT  = f'{MPPI_BASE}/fake_robot.py'


def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')

    # Resolved file paths (D* Lite side)
    dstar_params = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')

    # Launch configurations
    map_scenario_cfg = LaunchConfiguration('map_scenario')
    start_x_cfg      = LaunchConfiguration('start_x')
    start_y_cfg      = LaunchConfiguration('start_y')

    # Declare launch arguments
    declare_map_scenario = DeclareLaunchArgument(
        'map_scenario',
        default_value='open',
        description='Map scenario for dynamic_map_publisher.py '
                    '(e.g. open, wall, u_trap, corridor, enclosed).',
    )
    declare_start_x = DeclareLaunchArgument(
        'start_x',
        default_value='0.5',
        description='Robot start pose X coordinate (metres).',
    )
    declare_start_y = DeclareLaunchArgument(
        'start_y',
        default_value='0.5',
        description='Robot start pose Y coordinate (metres).',
    )

    # ── Global planner nodes ─────────────────────────────────────────────────

    # D* Lite global planner — publishes /global_plan (nav_msgs/Path)
    dstar_node = Node(
        package='dstar_navigation',
        executable='dstar_node',
        name='dstar_global_planner',
        parameters=[dstar_params],
        output='screen',
    )

    # Dynamic occupancy-grid publisher
    map_pub_node = Node(
        package='dstar_navigation',
        executable='dynamic_map_publisher.py',
        name='dynamic_map_publisher',
        parameters=[{
            'scenario': map_scenario_cfg,
            'start_x':  start_x_cfg,
            'start_y':  start_y_cfg,
        }],
        output='screen',
    )

    # Path simulator — moves robot pose along the published plan at 0.5 m/s
    path_sim_node = Node(
        package='dstar_navigation',
        executable='path_simulator.py',
        name='path_simulator',
        parameters=[{
            'start_x':      start_x_cfg,
            'start_y':      start_y_cfg,
            'linear_speed': 0.5,
        }],
        output='screen',
    )

    # Static map → odom transform (identity)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # ── Local controller nodes ───────────────────────────────────────────────

    # Fake robot simulator (Python process)
    fake_robot_proc = ExecuteProcess(
        cmd=['python3', FAKE_ROBOT],
        output='screen',
        name='fake_robot',
    )

    # Nav2 map server (empty map for controller costmap)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': MPPI_MAP,
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    # MPPI controller server
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[MPPI_PARAMS],
    )

    # Nav2 planner server (required by lifecycle manager)
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[MPPI_PARAMS],
    )

    # Nav2 lifecycle manager — autostarts map_server, controller, planner
    lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'controller_server',
                'planner_server',
            ],
        }],
    )

    # ── Shared visualisation — single RViz2 with D* Lite config ─────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        # Arguments
        declare_map_scenario,
        declare_start_x,
        declare_start_y,
        # Global planner
        dstar_node,
        map_pub_node,
        path_sim_node,
        static_tf_node,
        # Local controller
        fake_robot_proc,
        map_server_node,
        controller_node,
        planner_node,
        lifecycle_node,
        # Shared
        rviz_node,
    ])
