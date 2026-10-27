#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

GT_TOPIC_IN   = '/world/marsyard/pose/info'
ROVER_FRAME   = 'roar_rover'
GT_TOPIC_OUT  = '/ground_truth/pose'
ZED_TOPIC     = '/zed2i/zed_node/pose_with_covariance'


def generate_launch_description():
    loc_share  = get_package_share_directory('roar_localization')
    test_share = get_package_share_directory('roar_slam_testing')
    aruco_map_path = os.path.join(loc_share,  'config', 'aruco_map.yaml')
    scenario_path  = os.path.join(test_share, 'config', 'scenario_marsyard.yaml')

    ground_truth_bridge = Node(
        package='roar_localization',
        executable='ground_truth_bridge',
        name='ground_truth_bridge',
        output='screen',
        parameters=[{
            'ground_truth_topic': GT_TOPIC_IN,
            'rover_frame':        ROVER_FRAME,
            'output_topic':       GT_TOPIC_OUT,
            'position_variance':  1.0e-9,
            'use_sim_time':       True,
        }]
    )

    zed_pose_simulator = Node(
        package='roar_localization',
        executable='zed_pose_simulator.py',
        name='zed_pose_simulator',
        output='screen',
        parameters=[{
            'input_topic':       GT_TOPIC_OUT,
            'output_topic':      ZED_TOPIC,
            'pos_white_sigma':   0.002,
            'drift_walk_sigma':  0.0001,
            'drift_cap_m':       0.05,
            'yaw_white_sigma':   0.001,
            'reported_pos_var':  4.0e-6,
            'reported_yaw_var':  1.0e-6,
            'seed':              42,
            'use_sim_time':      True,
        }]
    )

    ieskf_node = Node(
        package='roar_localization',
        executable='ieskf_node',
        name='ieskf_node',
        output='screen',
        parameters=[{
            'frame_id':       'odom',
            'child_frame_id': 'base_footprint',
            'publish_tf':          True,       # ieskf owns odom->base_footprint (the fused
                                                # SLAM pose). Requires enable_odom_tf: false
                                                # on diff_drive_controller in roar_simulation's
                                                # diff_controller.yaml, otherwise both nodes
                                                # broadcast the same edge.
            'map_frame_id':        'map',
            'publish_map_to_odom': True,
            'odom_topic':     '/odom',
            'aruco_topic':    '/aruco/detections',
            'aruco_enabled':  False,
            'aruco_max_distance': 8.0,
            'aruco_map_path': aruco_map_path,
            'joint_left':  'wheel_lhs_mid_joint',
            'joint_right': 'wheel_rhs_mid_joint',
            'wheel_radius':     0.125,
            'wheelbase':        0.61,
            'pulses_per_rev':   201554.0,
            'max_delta_pulses': 500000.0,
            'max_vx':           2.0,
            # BNO055 is mounted on base_link, whose +X is rotated -90deg (yaw)
            # from the rover's true forward (see base_footprint_joint in
            # roar_simulation's rover.xacro). Correct it here so the fused
            # heading (and WIO dead-reckoning) matches true forward motion
            # instead of running 90deg perpendicular to the ZED position track.
            'imu_roll_deg':  0.0,
            'imu_pitch_deg': 0.0,
            'imu_yaw_deg':   -90.0,
            'cam_to_base_x': 0.0,
            'cam_to_base_y': 0.0,
            'cam_to_base_z': 0.0,
            'sigma_an':           27.5e-3,
            'sigma_aw':           18.6e-3,
            'sigma_ww':           3.3e-5,
            'sigma_pos':          0.002,
            'sigma_aruco_base':   0.01,
            'sigma_aruco_per_m':  0.005,
            'max_iterations':     10,
            'epsilon':            1e-6,
            'wvo_max_jump':    1.0,
            'wvo_window_size': 1,
            'wvo_w_wo':        0.0,
            'wvo_min_fill':    1,
            'max_omega_z':   1.5,
            'zed_cov_scale': 0.01,
            'zupt_enabled': True,
            'zupt_encoder_speed_threshold':    0.02,
            'zupt_angular_velocity_threshold': 0.05,
            'zupt_accel_variance_threshold':   0.05,
            'zupt_accel_buffer_size':          30,
            'zupt_min_stationary_duration':    0.5,
            'zupt_sigma_v':                    0.01,
            'use_sim_time': True,
        }],
        remappings=[
            ('/imu/data',     '/bno055/data'),
            ('/joint_states', '/joint_states_updated'),
        ],
    )

    # Empty heightmap_file → flat zero base map; live detections are still
    # verified per-obstacle and stamped once confirmed.
    terrain_mapping = Node(
        package='terrain_mapping',
        executable='terrain_mapping_node',
        name='terrain_mapping',
        output='screen',
        parameters=[{
            'map_resolution': 0.1,
            'map_width':  100.0,
            'map_height': 100.0,
            'map_origin_x': -50.0,
            'map_origin_y': -50.0,
            'heightmap_file': '',
            'occupancy_threshold': 0.3,
            'obstacle_topic': '/detections/obstacles',
            'odom_topic':     '/odom',
            'map_frame':      'map',
            'publish_rate':   2.0,
            'use_sim_time':   True,
        }]
    )

    slam_evaluator = Node(
        package='roar_slam_testing',
        executable='slam_evaluator',
        name='slam_evaluator',
        output='screen',
        parameters=[{
            'scenario_file':      scenario_path,
            'ground_truth_topic': GT_TOPIC_OUT,
            'odom_topic':         '/odom',
            'aruco_topic':        '/aruco/detections',
            'diagnostics_topic':  '/roar/ieskf_diagnostics',
            'occupancy_topic':    '/active_map/occupancy',
            'output_dir':         '',
            'use_sim_time':       True,
        }]
    )

    return LaunchDescription([
        ground_truth_bridge,
        zed_pose_simulator,
        ieskf_node,
        terrain_mapping,
        slam_evaluator,
    ])
