"""Standalone bringup for the MPPI local controller (Nav2 stack).

This launch file brings up all components needed to run the MPPI-based local
controller interactively, without any automated test harness.  It mirrors the
node set from /home/amrtamer/nav-stack_2026/empty_test/test_all.launch.py but
omits the test harness so the stack can be used for interactive / manual
operation.

The controller subscribes to /global_plan (nav_msgs/Path) and publishes
/cmd_vel, making it a drop-in companion to bringup_global_planner.launch.py.

Nodes / processes launched
--------------------------
- fake_robot             : Python process that simulates robot state
- map_server             : Serves the empty occupancy-grid map
- controller_server      : MPPI local controller (Nav2)
- planner_server         : Nav2 planner server (needed by lifecycle manager)
- lifecycle_manager      : Autostart manager for all Nav2 nodes
- rviz2                  : Visualisation with the MPPI rviz config

All absolute paths reuse the same locations as test_all.launch.py.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Absolute paths — kept in sync with test_all.launch.py
BASE   = '/home/amrtamer/nav-stack_2026/empty_test'
PARAMS = f'{BASE}/params/mppi_empty.yaml'
MAP    = f'{BASE}/maps/empty_map.yaml'
RVIZ   = f'{BASE}/rviz_config.rviz'
FAKE   = f'{BASE}/fake_robot.py'


def generate_launch_description():
    return LaunchDescription([

        # Fake robot simulator (Python process)
        ExecuteProcess(
            cmd=['python3', FAKE],
            output='screen',
            name='fake_robot',
        ),

        # Nav2 map server — publishes the empty occupancy grid
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': MAP,
                'topic_name': 'map',
                'frame_id': 'map',
            }],
        ),

        # Nav2 MPPI controller server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[PARAMS],
        ),

        # Nav2 planner server (required by lifecycle manager node_names list)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[PARAMS],
        ),

        # Nav2 lifecycle manager — autostarts map_server, controller, planner
        Node(
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
        ),

        # RViz2 — use the MPPI-specific config from the empty_test workspace
        ExecuteProcess(
            cmd=['rviz2', '-d', RVIZ],
            output='log',
            name='rviz2',
        ),
    ])
