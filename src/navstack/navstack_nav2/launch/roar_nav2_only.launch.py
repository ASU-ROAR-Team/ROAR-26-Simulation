import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



GT_TOPIC_IN   = '/world/rover_world/pose/info'
ROVER_FRAME   = 'roar_rover'
ZED_TOPIC     = '/zed2i/zed_node/pose_with_covariance'

def generate_launch_description():
    pkg_nav    = get_package_share_directory('navstack_nav2')
    nav_params = os.path.join(pkg_nav, 'config', 'roar_nav2_sim.yaml')
    rviz_config = os.path.join(pkg_nav, 'config', 'roar_nav2.rviz')

    # ─────────────────────────────────────────────────────────────────────
    # SLAM INTEGRATION POINT
    # This launches SLAM against the SAME sim the SLAM package already
    # targets (basic_rover.launch.py) — not the separate synthetic-IMU/
    # PoseArray dev setup in simulation.launch.py.
    # The sim itself already provides:
    #   /bno055/data                 (real IMU, bridged directly)
    #   /joint_states_updated        (encoder pulses, via encoder_sim_node.py)
    #   /world/rover_world/pose/info (ground truth, tf2_msgs/TFMessage —
    #                                 Ignition publishes ground truth under
    #                                 the world file's internal SDF <world
    #                                 name>, not its filename. The world
    #                                 files under ROAR-26-Simulation/
    #                                 dev_environment/worlds/ (world1/2/3)
    #                                 all declare name="rover_world" —
    #                                 verify with `grep 'world name=' <file>`
    #                                 if you point this at a different world)
    # so no synthetic IMU / pose-array bridge / noise-injection node is
    # needed here — only ground_truth_bridge, which turns the ground-truth
    # TF into the fake ZED pose IESKF fuses (the sim has no ZED pose topic
    # of its own). Keep this in sync with roar_slam_testing/slam_test.launch.py.
    loc_share  = get_package_share_directory('roar_localization')
    test_share = get_package_share_directory('roar_slam_testing')

    aruco_map_path = os.path.join(loc_share,  'config', 'aruco_map.yaml')
    scenario_path  = os.path.join(test_share, 'config', 'scenario_marsyard.yaml')

    pkg_perception = get_package_share_directory('perception_module')

    # ── Ground truth bridge (Gazebo TF → fake ZED pose, direct — no separate
    #    noise-injection stage; same single bridge the stack always used) ────
    ground_truth_bridge = Node(
        package='roar_localization',
        executable='ground_truth_bridge',
        name='ground_truth_bridge',
        output='screen',
        parameters=[{
            'ground_truth_topic': GT_TOPIC_IN,
            'rover_frame':        ROVER_FRAME,
            'output_topic':       ZED_TOPIC,
            'position_variance':  1.0e-9,
            'use_sim_time':       True,
        }]
    )

    # ── IESKF localization ────────────────────────────────────────────────────
    ieskf_node = Node(
        package='roar_localization',
        executable='ieskf_node',
        name='ieskf_node',
        output='screen',
        parameters=[{
            'frame_id':       'odom',
            'child_frame_id': 'base_footprint',
            'publish_tf':          True,       # ieskf owns odom->base_footprint (the fused
                                                # SLAM pose) so Nav2's TF-based costmap/planner
                                                # lookups track it instead of diff_drive_
                                                # controller's raw wheel odometry. Requires
                                                # enable_odom_tf: false on diff_drive_controller
                                                # in roar_simulation's diff_controller.yaml —
                                                # otherwise both nodes broadcast the same edge.
            'map_frame_id':        'map',
            'publish_map_to_odom': True,
            'odom_topic':     '/odom',
            'aruco_topic':    '/aruco/detections',
            'aruco_enabled':  True,
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

    # ── Perception stack (LiDAR clustering → /detections/obstacles, ArUco, etc.) ──
    # Feeds terrain_mapping below via /detections/obstacles.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # ── Terrain mapping module ────────────────────────────────────────────────
    # heightmap_mars_yard.npz is the correct bake for the mars_yard terrain.
    # Loaded from terrain_mapping's installed share dir (not a path computed
    # from the workspace source tree) so this launch file works on any
    # machine/workspace layout after a colcon build.
    pkg_terrain_mapping = get_package_share_directory('terrain_mapping')
    heightmap_file = os.path.join(
        pkg_terrain_mapping, 'working_heightmaps', 'heightmap_mars_yard.npz'
    )

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
            'heightmap_file': heightmap_file,
            'occupancy_threshold': 0.3,
            'obstacle_topic': '/detections/obstacles',
            'odom_topic':     '/odom',
            'map_frame':      'map',
            'publish_rate':   2.0,
            'use_sim_time':   True,
        }]
    )

    # ── Heightmap to Costmap Converter ───────────────────────────────────────
    # Feeds /terrain_costmap, consumed by DStarPlanner's edgeCost() alongside
    # its own slope term. heightmap_file above is '' (flat zero base map), so
    # until a real .npz is supplied here this node runs but has no elevation
    # variation to derive roughness/steepness from.
    heightmap_to_costmap_node = Node(
        package='dstar_navigation',
        executable='heightmap_to_costmap.py',
        name='heightmap_to_costmap',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}]
    )

    # ── Evaluator ─────────────────────────────────────────────────────────────
    slam_evaluator = Node(
        package='roar_slam_testing',
        executable='slam_evaluator',
        name='slam_evaluator',
        output='screen',
        parameters=[{
            'scenario_file':      scenario_path,
            'ground_truth_topic': ZED_TOPIC,
            'odom_topic':         '/odom',
            'aruco_topic':        '/aruco/detections',
            'diagnostics_topic':  '/roar/ieskf_diagnostics',
            'occupancy_topic':    '/active_map/occupancy',
            'output_dir':         '',     # auto: ~/slam_eval_<timestamp>/
            'use_sim_time':       True,
        }]
    )

    # NOTE: map_server loads a static map from disk (yaml_filename in the
    # params file). It is currently configured with an empty filename, so it
    # publishes nothing. Once SLAM is publishing /map, this node is redundant
    # and can be removed — along with 'map_server' from node_names below.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}]
    )

    # D* Lite global planner (plugin: nav2_dstar_planner/DStarPlanner)
    # Blocks until map→odom TF and /map exist.
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}]
    )

    # MPPI local controller (plugin: nav2_mppi_controller::MPPIController)
    # Publishes /cmd_vel, remapped to the DiffDriveController's input topic.
    #
    # NOTE 2026-07-16: cmd_vel_correction_node (a Rz(-90deg) rotation of
    # linear.x/y, meant to mirror diff_drive_odom_correction_node) was wired
    # in here and reverted. diff_drive_controller only ever reads
    # linear.x — it has no lateral DOF. Rotating (linear.x, linear.y) by a
    # non-180-multiple angle bleeds part or all of linear.x into the ignored
    # linear.y field; for a pure forward command it zeroed linear.x
    # entirely, so the robot stopped translating altogether (worse than the
    # uncorrected behavior it was meant to fix). The underlying wheel-axis
    # issue this was trying to work around is now fixed at its source (see
    # ieskf_node's imu_yaw_deg above), so no cmd_vel-side correction is
    # needed at all.
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # Recovery behaviors: spin, backup, drive_on_heading, wait
    # NOTE: previously had no cmd_vel remap at all (defaulted to the
    # unconnected literal '/cmd_vel' topic — recovery behaviors like
    # spin/backup were never wired to the robot).
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    # Behavior tree orchestrator — exposes the /navigate_to_pose action
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav_params, {'use_sim_time': True}]
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'map_server',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    return LaunchDescription([
        ground_truth_bridge,
        ieskf_node,
        perception,
        terrain_mapping,
        heightmap_to_costmap_node,
        slam_evaluator,
        map_server,
        planner,
        controller,
        behavior_server,
        bt_navigator,
        lifecycle,
    ])
