#!/usr/bin/env python3
"""
simulation.launch.py
====================
Simulation launch file for the IESKF localization stack.

What this launches:
  - ieskf_node                  — the filter itself
  - aruco_simulator_node        — fakes ArUco detections from Gazebo ground truth
  - ground_truth_bridge         — Gazebo PoseArray → fake ZED pose
  - synthetic_imu_node          — differentiates ground truth → fake IMU

This launch does NOT start Gazebo or the rover model — those come from the
simulation team's launch file (roar_simulation). Bring this stack up AFTER
Gazebo is running and publishing /world/marsyard/pose/info.

The two helper nodes (ground_truth_bridge, synthetic_imu_node) come from
the existing roar_localization package — same as before integration of
ArUco + ZUPT.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('roar_localization')
    aruco_map_path = os.path.join(pkg_share, 'config', 'aruco_map.yaml')

    # ── Synthetic IMU (Gazebo pose → /imu/data) ────────────────────────────────
    synthetic_imu = Node(
        package='roar_localization',
        executable='synthetic_imu_node',
        name='synthetic_imu_node',
        output='screen',
        parameters=[{
            'ground_truth_topic': '/world/marsyard/pose/info',
            'model_name': 'roar_rover',
            'pose_index': 11,
            'ema_alpha': 0.50,
            'use_sim_time': False,
        }]
    )

    # ── Ground truth bridge (Gazebo pose → /zed2i pose) ────────────────────────
    ground_truth_bridge = Node(
        package='roar_localization',
        executable='ground_truth_bridge',
        name='ground_truth_bridge',
        output='screen',
        parameters=[{
            'ground_truth_topic': '/world/marsyard/pose/info',
            'model_name': 'roar_rover',
            'pose_index': 11,
            'output_topic': '/zed2i/zed_node/pose_with_covariance',
            'use_sim_time': False,
        }]
    )

    # ── ArUco simulator (Gazebo pose → /aruco/detections) ──────────────────────
    aruco_simulator = Node(
        package='roar_localization',
        executable='aruco_simulator_node',
        name='aruco_simulator',
        output='screen',
        parameters=[{
            'ground_truth_topic': '/world/marsyard/pose/info',
            'aruco_topic':        '/aruco/detections',
            'model_name':         'roar_rover',
            'pose_index':         11,
            'aruco_map_path':     aruco_map_path,
            # Camera mount (same as IESKF node — must match)
            'cam_to_base_x':      0.0,
            'cam_to_base_y':      0.0,
            'cam_to_base_z':      0.0,
            'cam_pitch_deg':      0.0,
            # Camera intrinsics (ZED2i nominal — wide FOV)
            'horizontal_fov_deg': 90.0,
            'vertical_fov_deg':   60.0,
            # Range
            'min_distance':        0.3,
            'max_distance':       15.0,
            # Noise
            'noise_base':          0.005,
            'noise_per_meter':     0.003,
            'noise_seed':          42,
            # Detection rate
            'detection_rate_hz':   10.0,
            'use_sim_time':        False,
        }]
    )

    # ── IESKF node ────────────────────────────────────────────────────────────
    ieskf_node = Node(
        package='roar_localization',
        executable='ieskf_node',
        name='ieskf_node',
        output='screen',
        parameters=[{
            # Frames + topics
            'frame_id':       'odom',
            'child_frame_id': 'base_footprint',
            'publish_tf':     True,            # ieskf owns odom->base_footprint (the fused
                                                # SLAM pose). Requires enable_odom_tf: false
                                                # on diff_drive_controller in roar_simulation's
                                                # diff_controller.yaml, otherwise both nodes
                                                # broadcast the same edge.
            'map_frame_id':        'map',
            'publish_map_to_odom': True,       # static identity; roots /map above /odom
                                                # (published on /tf_static — safe regardless
                                                # of which clock domain odom->base_footprint
                                                # is stamped in, see ieskf_node.hpp)
            'odom_topic':     '/odom',
            'aruco_topic':    '/aruco/detections',
            'aruco_enabled':  True,
            'aruco_max_distance': 15.0,
            'aruco_map_path': aruco_map_path,

            # Drivetrain (marsyard sim values)
            'joint_left':  'wheel_lhs_mid_joint',
            'joint_right': 'wheel_rhs_mid_joint',
            'wheel_radius':     0.13,
            'wheelbase':        0.709,
            'pulses_per_rev':   6.2831853,     # Gazebo joint_states in radians
            'max_delta_pulses': 50.0,
            'max_vx':           2.0,

            # IMU mounting: the BNO055 is bolted to base_link, but base_link's
            # own +X is rotated -90deg (yaw) from the rover's true forward —
            # see urdf/base/rover.xacro's base_footprint_joint in roar_simulation.
            # This rotates the IMU orientation/accel/gyro into the true
            # X-forward body frame (base_footprint) before fusion, so WIO
            # heading and the ZED position track agree.
            'imu_roll_deg':  0.0,
            'imu_pitch_deg': 0.0,
            'imu_yaw_deg':   -90.0,

            # ZED extrinsic (sim: place camera 20 cm forward, 30 cm up)
            'cam_to_base_x': 0.0,
            'cam_to_base_y': 0.0,
            'cam_to_base_z': 0.0,

            # IESKF noise (matched to Gazebo URDF xacro noise model)
            'sigma_an':           27.5e-3,
            'sigma_aw':           18.6e-3,
            'sigma_ww':           3.3e-5,
            'sigma_pos':          1.0e-4,      # near-perfect GT in sim
            'sigma_aruco_base':   0.01,
            'sigma_aruco_per_m':  0.005,
            'max_iterations':     10,
            'epsilon':            1e-6,

            # WVO (sim: passthrough, no smoothing)
            'wvo_max_jump':    1.0,
            'wvo_window_size': 1,
            'wvo_w_wo':        0.0,
            'wvo_min_fill':    1,

            # FIS ranges
            'max_omega_z':   1.5,
            'zed_cov_scale': 0.01,

            # ZUPT (enabled for sim validation)
            'zupt_enabled': True,
            'zupt_encoder_speed_threshold':    0.02,
            'zupt_angular_velocity_threshold': 0.05,
            'zupt_accel_variance_threshold':   0.05,
            'zupt_accel_buffer_size':          30,
            'zupt_min_stationary_duration':    0.5,
            'zupt_sigma_v':                    0.01,

            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        synthetic_imu,
        ground_truth_bridge,
        aruco_simulator,
        ieskf_node,
    ])
