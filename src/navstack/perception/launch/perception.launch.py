"""
perception.launch.py
====================
Launches the full perception stack for the ROAR autonomous chair.

Active nodes
────────────
  lidar_cluster_node      — ZED point cloud → AV-grade filtering → DBSCAN clusters
  aruco_detector_node     — ZED left image  → ArUco ID + 6-DOF pose
  obstacle_publisher_node — LiDAR clusters  → world-frame persistent obstacle map
  bev_visualizer_node     — Bird's-Eye-View visualisation (optional, for RViz debug)

Kept but NOT launched (left in source for reference)
────────────────────────────────────────────────────
  camera_detector_node    — YOLO object classifier (not needed for obstacle avoidance)
  frustum_fusion_node     — LiDAR-camera frustum fusion (requires YOLO)

All tunable thresholds live in
    config/perception_params.yaml
Edit that file and re-launch — no recompile needed.

Launch arguments
────────────────
  use_sim_time:=true   — sync to bag /clock (default: false)
  rviz:=true           — open RViz2 with perception config (default: false)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('perception_module')
    params_file = os.path.join(pkg_share, 'config', 'perception_params.yaml')

    # RViz config — use the one in home dir if it exists
    rviz_config = os.path.expanduser('~/perception_pipeline.rviz')

    # ── Launch arguments ─────────────────────────────────────────────────

    # use_sim_time: set true when playing from a bag
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock from bag playback (set true when replaying bags)'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # rviz: open RViz2 with the perception pipeline config
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 with perception pipeline config'
    )

    # ── LiDAR topic override (real vs simulation) ───────────────────────
    # Pass lidar_topic:=/real/topic at the command line to switch sensors.
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/zed2i/points_updated',
        description='Input PointCloud2 topic (sim or real ZED)'
    )
    lidar_topic = LaunchConfiguration('lidar_topic')

    # ── LiDAR Cluster Node ───────────────────────────────────────────────
    # ERC Mars pipeline:
    #   GPU ROI → Voxel → SOR → ROR → Slope-aware RANSAC → Roughness filter
    #   → DBSCAN → BBox + Aspect-ratio + Distance + Height filters
    # Publishes: /detection/lidar_clusters, /lidar/points_filtered,
    #            /lidar/points_no_ground, /visualization/cluster_markers
    lidar_node = Node(
        package='perception_module',
        executable='lidar_cluster_node',
        name='lidar_cluster_node',
        output='screen',
        parameters=[params_file, {
            'use_sim_time': use_sim_time,
            'lidar_topic':  lidar_topic,
        }],
    )

    # ── ArUco Detector Node ──────────────────────────────────────────────
    # Detects ArUco markers in the ZED left camera and publishes full
    # 6-DOF pose (position + quaternion) and marker IDs.
    # Publishes: /detection/aruco_3d, /detection/aruco_poses,
    #            /visualization/aruco_markers, /detection/aruco_image
    aruco_node = Node(
        package='perception_module',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── Obstacle Publisher Node ──────────────────────────────────────────
    # Reads LiDAR clusters, transforms to world/odom frame, and maintains
    # a persistent obstacle map with EMA smoothing and stale-obstacle pruning.
    # Publishes: /planning/obstacles, /planning/obstacle_markers
    obstacle_node = Node(
        package='perception_module',
        executable='obstacle_publisher_node',
        name='obstacle_publisher_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── BEV Visualizer Node (debug only) ────────────────────────────────
    # Generates a bird's-eye-view image of the point cloud for RViz.
    # Publishes: /visualization/bev_image
    bev_node = Node(
        package='perception_module',
        executable='bev_visualizer_node',
        name='bev_visualizer_node',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    # ── RViz2 (optional) ─────────────────────────────────────────────────
    # Only launched when rviz:=true is passed.
    rviz_cmd = (['-d', rviz_config] if os.path.isfile(rviz_config) else [])
    rviz_node = ExecuteProcess(
        cmd=['rviz2'] + rviz_cmd,
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        lidar_topic_arg,
        lidar_node,
        aruco_node,
        obstacle_node,
        bev_node,
        rviz_node,
    ])