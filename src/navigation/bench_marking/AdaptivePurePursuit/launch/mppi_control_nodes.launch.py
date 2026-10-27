import os
import datetime
import subprocess
import time
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    LogInfo,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    test_arg = context.launch_configurations.get('test', '')
    test_case = test_arg if test_arg else 'all'

    use_costmap = str(context.launch_configurations.get('use_costmap', 'false')).lower() in ('1', 'true', 'yes', 'on')

    custom_path = context.launch_configurations.get('custom_path', '')
    path_arg = context.launch_configurations.get('path', '')
    if path_arg:
        custom_path = path_arg

    scenario = str(context.launch_configurations.get('scenario', '') or '').lower()
    scenario_seed = context.launch_configurations.get('scenario_seed', '0')
    map_width = int(context.launch_configurations.get('map_width', 169))
    map_height = int(context.launch_configurations.get('map_height', 114))
    map_resolution = float(context.launch_configurations.get('map_resolution', 0.25))

    record_bag = str(
        context.launch_configurations.get('record_bag', 'true')
    ).lower() in ('1', 'true', 'yes', 'on')

    pkg_share_control = get_package_share_directory('adaptive_pure_pursuit')

    rviz_config = os.path.join(
        pkg_share_control,
        'rviz',
        'validation_config.rviz'
    )

    urdf_file = os.path.join(
        pkg_share_control,
        'urdf',
        'robot.urdf'
    )

    bag_dir = os.path.join(
        '/tmp',
        f'mppi_benchmarking_bag_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )

    def _cleanup_stale_nodes():
        subprocess.run(
            [
                'bash',
                '-lc',
                (
                    "pkill -9 -f nav2_bench_bridge; "
                    "pkill -9 -f test_orchestrator; "
                    "pkill -9 -f control_monitor; "
                    "pkill -9 -f rviz2; "
                    "pkill -9 -f rviz_visualizer; "
                    "true"
                )
            ],
            check=False,
        )
        time.sleep(1)
        print('Pre-launch cleanup complete: stale benchmark nodes terminated.')

    _cleanup_stale_nodes()

    # Launch action to wait for controller_server while nodes (and TF broadcast) are actively running
    wait_for_controller_action = ExecuteProcess(
        cmd=[
            'python3', '-c',
            'import time, sys, rclpy; '
            'from lifecycle_msgs.srv import GetState; '
            'rclpy.init(); node = rclpy.create_node("benchmark_startup_waiter"); '
            'client = node.create_client(GetState, "/controller_server/get_state"); '
            'start = time.time(); success = False; '
            'while time.time() - start < 30.0: '
            '    if client.wait_for_service(timeout_sec=1.0): '
            '        future = client.call_async(GetState.Request()); '
            '        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0); '
            '        if future.result() is not None and future.result().current_state.label == "active": '
            '            print("controller_server is ACTIVE — proceeding."); success = True; break; '
            '    time.sleep(0.5); '
            'node.destroy_node(); rclpy.shutdown(); '
            'if not success: '
            '    print("ERROR: controller_server never reached ACTIVE state."); sys.exit(1)'
        ],
        output='screen'
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'record',
            '-o',
            bag_dir,
            '/path',
            '/odometry/filtered',
            '/cmd_vel'
        ],
        output='screen',
        condition=IfCondition(str(record_bag))
    )

    stop_rosbag_action = ExecuteProcess(
        cmd=[
            'pkill',
            '-f',
            'ros2 bag record'
        ],
        output='screen'
    )

    generate_report_action = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                pkg_share_control,
                'generate_report.py'
            )
        ],
        output='screen'
    )

    bridge_node = Node(
        package='adaptive_pure_pursuit',
        executable='nav2_bench_bridge',
        name='nav2_bench_bridge',
        output='screen',
        parameters=[
            {
                'odom_topic': '/odometry/filtered',
                'path_topic': '/path',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint'
            }
        ]
    )

    test_orchestrator_node = Node(
        package='adaptive_pure_pursuit',
        executable='test_orchestrator',
        name='test_orchestrator',
        output='screen',
        parameters=[
            {
                'test_case': test_case,
                'custom_path': custom_path
            }
        ]
    )

    metrics_node = Node(
        package='adaptive_pure_pursuit',
        executable='control_monitor',
        name='control_monitor',
        output='screen',
        parameters=[
            {
                'csv_output_path': '/tmp/mppi_controller_metrics.csv',
                'plot_output_path': '/tmp/mppi_controller_metrics.png',
                'publish_rate_hz': 50.0,
                'overshoot_threshold_m': 0.2,
                'velocity_limit_m_s': 1.0,
                'angular_velocity_limit_rad_s': 1.5,
                'path_topic': '/path',
                'odom_topic': '/odometry/filtered',
                'cmd_topic': '/cmd_vel',
                'case_topic': '/testing/current_case'
            }
        ]
    )

    rviz_visualizer_node = Node(
        package='adaptive_pure_pursuit',
        executable='rviz_visualizer',
        name='rviz_visualizer',
        output='screen'
    )

    costmap_node = None
    if use_costmap:
        if scenario:
            costmap_node = Node(
                package='adaptive_pure_pursuit',
                executable='obstacle_generator',
                name='obstacle_generator',
                output='screen',
                parameters=[
                    {
                        'scenario': LaunchConfiguration('scenario'),
                        'seed': int(scenario_seed),
                        'map_width': int(map_width),
                        'map_height': int(map_height),
                        'resolution': float(map_resolution),
                        'frame_id': 'odom',
                        'publish_rate_hz': 1.0,
                    }
                ]
            )
        else:
            costmap_node = Node(
                package='adaptive_pure_pursuit',
                executable='costmap_publisher',
                name='local_costmap',
                output='screen',
                parameters=[
                    os.path.join(pkg_share_control, 'config', 'local_costmap.yaml')
                ]
            )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': open(urdf_file, 'r').read()
            }
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            rviz_config
        ]
    )

    orchestrator_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=test_orchestrator_node,
            on_exit=[
                stop_rosbag_action,
                TimerAction(
                    period=3.0,
                    actions=[
                        generate_report_action
                    ]
                ),
                LogInfo(
                    msg="======== MPPI BENCHMARKING SEQUENCE FINISHED, REPORT GENERATED ========"
                ),
                LogInfo(
                    msg="REQUESTING LAUNCH SHUTDOWN"
                ),
                Shutdown()
            ]
        )
    )

    launch_shutdown_logger = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                LogInfo(msg='LAUNCH SHUTDOWN COMPLETE')
            ]
        )
    )

    actions = [
        bridge_node,
        test_orchestrator_node,  # Launches orchestrator (starts TF broadcaster immediately)
        wait_for_controller_action,  # Non-blocking waiter runs alongside TF broadcast
    ]

    if use_costmap and scenario:
        actions.append(
            LogInfo(
                msg=(
                    f"Launching obstacle_generator with scenario={scenario}, "
                    f"seed={scenario_seed}, map_width={map_width}, map_height={map_height}, "
                    f"map_resolution={map_resolution}"
                )
            )
        )

    if costmap_node is not None:
        actions.append(costmap_node)

    actions.extend([
        metrics_node,
        rviz_visualizer_node,
        robot_state_publisher_node,
        rviz_node,
        rosbag_record,
        orchestrator_exit_handler,
        launch_shutdown_logger,
    ])

    return actions


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'test',
                default_value='',
                description="Empty (default) = run ALL test cases."
            ),
            DeclareLaunchArgument(
                'custom_path',
                default_value='',
                description="Optional user supplied path."
            ),
            DeclareLaunchArgument(
                'record_bag',
                default_value='true'
            ),
            DeclareLaunchArgument(
                'use_costmap',
                default_value='false',
                description='If true, publish static local costmap and enable costmap-aware MPPI critics'
            ),
            DeclareLaunchArgument(
                'scenario',
                default_value='',
                description='If set and use_costmap:=true, run the named obstacle scenario (empty uses static costmap)'
            ),
            DeclareLaunchArgument(
                'scenario_seed',
                default_value='0',
                description='Random seed for obstacle scenario generator'
            ),
            DeclareLaunchArgument(
                'map_width',
                default_value='169',
                description='Width (cells) of generated scenario map'
            ),
            DeclareLaunchArgument(
                'map_height',
                default_value='114',
                description='Height (cells) of generated scenario map'
            ),
            DeclareLaunchArgument(
                'map_resolution',
                default_value='0.25',
                description='Resolution (m/cell) of generated scenario map'
            ),
            DeclareLaunchArgument(
                'path',
                default_value='',
                description='Optional external mission path file; alias for custom_path'
            ),
            OpaqueFunction(
                function=launch_setup
            )
        ]
    )