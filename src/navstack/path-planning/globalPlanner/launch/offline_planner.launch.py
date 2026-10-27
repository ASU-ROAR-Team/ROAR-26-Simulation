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

    # Declare Launch Arguments for Heightmap, Origin & Calibration parameters
    heightmap_png_arg = DeclareLaunchArgument(
        'heightmap_png',
        default_value='heightmap.png',
        description='Path to input heightmap grayscale image file'
    )
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Map resolution in meters per pixel'
    )
    min_height_arg = DeclareLaunchArgument(
        'min_height',
        default_value='0.0',
        description='Minimum elevation Z in meters'
    )
    max_height_arg = DeclareLaunchArgument(
        'max_height',
        default_value='1.5',
        description='Maximum elevation Z in meters'
    )
    origin_x_arg = DeclareLaunchArgument(
        'origin_x',
        default_value='0.0',
        description='World frame origin X position in meters'
    )
    origin_y_arg = DeclareLaunchArgument(
        'origin_y',
        default_value='0.0',
        description='World frame origin Y position in meters'
    )
    center_origin_arg = DeclareLaunchArgument(
        'center_origin',
        default_value='true',
        description='Auto-center map (0,0) around middle of image if origin_x/y are 0'
    )
    max_safe_slope_arg = DeclareLaunchArgument(
        'max_safe_slope_deg',
        default_value='25.0',
        description='Maximum traversable slope angle in degrees'
    )
    max_safe_step_arg = DeclareLaunchArgument(
        'max_safe_step_m',
        default_value='0.30',
        description='Maximum traversable vertical step height in meters'
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
            'heightmap_png': LaunchConfiguration('heightmap_png'),
            'resolution': LaunchConfiguration('resolution'),
            'min_height': LaunchConfiguration('min_height'),
            'max_height': LaunchConfiguration('max_height'),
            'origin_x': LaunchConfiguration('origin_x'),
            'origin_y': LaunchConfiguration('origin_y'),
            'center_origin': LaunchConfiguration('center_origin'),
            'max_safe_slope_deg': LaunchConfiguration('max_safe_slope_deg'),
            'max_safe_step_m': LaunchConfiguration('max_safe_step_m'),
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
        heightmap_png_arg,
        resolution_arg,
        min_height_arg,
        max_height_arg,
        origin_x_arg,
        origin_y_arg,
        center_origin_arg,
        max_safe_slope_arg,
        max_safe_step_arg,
        dstar_node,
        offline_planner_node,
        static_tf_node,
        rviz_node,
        Node(
                    package='dstar_navigation',
                    executable='heightmap_to_costmap.py',
                    name='heightmap_to_costmap',
                    parameters=[{
                        'use_sim_time': False,
                        'heightmap_topic': '/active_map/heightmap',
                        'heightmap_range_topic': '/active_map/heightmap_range',
                        'unified_costmap_topic': '/unified_costmap',
                        'MAX_SAFE_SLOPE': 1.0, #45 degrees for now
                        'MAX_SAFE_ROUGHNESS': 1.0,
                        'steepness_weight': 0.5,
                        'roughness_weight': 0.05,
                        'max_traversable_height_diff': 0.35,
                        'publish_rate': 2.0,
                        'inscribed_radius': 0.35,
                        'inflation_radius': 0.8,
                        'inflation_cost_scaling': 9.0
                    }]
                ),
    ])