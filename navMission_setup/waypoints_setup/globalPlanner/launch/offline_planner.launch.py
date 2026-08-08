"""Offline Path Generation: D* Lite global planner + Offline Sequence Script.

This launch file isolates the global planner to run against heightmaps 
and waypoints without launching the local MPPI controller or live simulator.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')

    # Resolved file paths
    dstar_params = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config  = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')

    # Declare Launch Arguments for waypoints/heightmap file selection
    waypoints_file_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value='waypoints.csv',
        description='Waypoints CSV filename or path'
    )

    heightmap_file_arg = DeclareLaunchArgument(
        'heightmap_file',
        default_value='heightmap_world.npz',
        description='Heightmap NPZ filename or path'
    )

    # ── Global planner nodes ─────────────────────────────────────────────────

    # D* Lite global planner — Action Server for path generation
    dstar_node = Node(
        package='dstar_navigation',
        executable='dstar_node',
        name='dstar_global_planner',
        parameters=[dstar_params],
        output='screen',
    )

    # Offline Sequence Planner — Loads heightmap.png, publishes maps, teleports odom, requests paths
    offline_planner_node = Node(
        package='dstar_navigation',
        executable='offline_sequence_planner.py',
        name='offline_sequence_planner',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
            'heightmap_file': LaunchConfiguration('heightmap_file'),
        }],
        output='screen',
    )

    # Static map → odom transform
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    # ── Visualisation ────────────────────────────────────────────────────────
    
    # RViz2 to watch the sequence generation live
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        waypoints_file_arg,
        heightmap_file_arg,
        dstar_node,
        offline_planner_node,
        static_tf_node,
        rviz_node,
    ])