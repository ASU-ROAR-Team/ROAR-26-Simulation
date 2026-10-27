#!/usr/bin/env python3
"""
ieskf_launch.py
===============
Real rover launch file for IESKF localization with ArUco landmark updates
and ZUPT (Zero Velocity Updates).

For the actual rover with:
  - BNO055 IMU (NDOF mode)        → /imu/data
  - Wheel encoders (one per side)  → /joint_states
  - ZED2i camera                   → /zed2i/zed_node/pose_with_covariance
  - Perception ArUco detector      → /aruco/detections (roar_msgs/ArucoDetectionArray)

Output:
  /roar/odom_ieskf (Odometry)
  TF: odom → base_link
  TF: map  → odom       (static identity; roots /map in the TF tree)

Calibration steps BEFORE first drive:
  1. imu_yaw_deg       — measure with rover on known heading
  2. cam_to_base_x/y/z — measure ZED offset from CAD
  3. pulses_per_rev    — verify encoder format (radians or counts)
  4. aruco_map_path    — verify path; populate with Update Report #3 positions
  5. Verify ArUco topic name matches perception team's publication
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('roar_localization')
    aruco_map_path = os.path.join(pkg_share, 'config', 'aruco_map.yaml')

    ieskf_node = Node(
        package='roar_localization',
        executable='ieskf_node',
        name='ieskf_node',
        output='screen',
        parameters=[{
            # Frame IDs and topics
            'frame_id':       'odom',
            'child_frame_id': 'base_link',
            'publish_tf':     True,                  # real rover: no conflict
            'map_frame_id':        'map',
            'publish_map_to_odom': True,              # static identity; roots /map above /odom
            'odom_topic':     '/odom',
            'aruco_topic':    '/aruco/detections',
            'aruco_enabled':  True,
            'aruco_max_distance': 15.0,
            'aruco_map_path': aruco_map_path,

            # Joint names (verify with `ros2 topic echo /joint_states --once`)
            'joint_left':  'wheel_lhs_mid_joint',
            'joint_right': 'wheel_rhs_mid_joint',

            # Drivetrain — measure from rover CAD / hardware
            'wheel_radius':     0.13,
            'wheelbase':        0.709,
            'pulses_per_rev':   6.2831853,           # 2π if radians; PPR×4 if counts
            'max_delta_pulses': 50.0,
            'max_vx':           2.0,

            # IMU mounting offset (MEASURE with rover on known heading)
            'imu_roll_deg':  0.0,
            'imu_pitch_deg': 0.0,
            'imu_yaw_deg':   0.0,                    # ← MEASURE

            # ZED2i extrinsic (MEASURE from CAD)
            'cam_to_base_x': 0.0,                    # ← MEASURE
            'cam_to_base_y': 0.0,                    # ← MEASURE
            'cam_to_base_z': 0.0,                    # ← MEASURE
            'cam_pitch_deg': 0.0,                    # if camera tilted down

            # IESKF noise (real BNO055 + ZED2i; tune from Allan variance later)
            'sigma_an':           3.0e-3,
            'sigma_aw':           1.0e-4,
            'sigma_ww':           8.7e-5,
            'sigma_pos':          1.0e-3,            # real ZED noise
            'sigma_aruco_base':   0.01,              # ArUco at close range
            'sigma_aruco_per_m':  0.005,             # noise growth with distance
            'max_iterations':     10,
            'epsilon':            1e-6,

            # WVO (real encoders help on rough terrain)
            'wvo_max_jump':    0.5,
            'wvo_window_size': 3,
            'wvo_min_fill':    2,
            'wvo_w_wo':        0.2,                  # 20% encoder, 80% visual

            # FIS ranges
            'max_omega_z':   1.5,
            'zed_cov_scale': 0.1,                    # real ZED cov scale

            # ZUPT
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
        ieskf_node,
    ])
