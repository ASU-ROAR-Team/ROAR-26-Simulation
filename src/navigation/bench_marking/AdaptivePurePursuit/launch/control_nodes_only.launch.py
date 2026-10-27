import os
import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, Shutdown, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve launch arguments
    # 'test' : '' (default) -> run ALL test cases
    #          'TC-03' or 'TC-03,TC-07,TC-10' -> run only the specified test case(s)
    # 'custom_path' : '' (default, unused)
    #          'x1,y1;x2,y2;x3,y3' -> run a single user-supplied path instead of any
    #          built-in test case. Takes priority over 'test' if both are given.
    test_arg = context.launch_configurations.get('test', '')
    test_case = test_arg if test_arg else 'all'
    custom_path = context.launch_configurations.get('custom_path', '')
    record_bag = str(context.launch_configurations.get('record_bag', 'true')).lower() in ('1', 'true', 'yes', 'on')
    
    pkg_share = get_package_share_directory('adaptive_pure_pursuit')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'validation_config.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    # Setup Paths
    bag_dir = os.path.join(
        '/tmp',
        f'adaptive_pure_pursuit_bag_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}',
    )

    # --- DYNAMIC PATH FIX FOR MULTIPLE LAPTOPS ---
    home_dir = os.path.expanduser('~')

    # 1. Define Actions (Bagging, Cleanup)
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', bag_dir, '/path', '/odometry/filtered', '/cmd_vel'],
        output='screen',
        condition=IfCondition(str(record_bag))
    )

    stop_rosbag_action = ExecuteProcess(
        cmd=['pkill', '-f', 'ros2 bag record'],
        output='screen',
    )

    # 2. Define Nodes
    controller_node = Node(
        package='adaptive_pure_pursuit',
        executable='adaptiveLd_fixed',
        name='controller',
        output='screen',
        parameters=[config_file]
    )

    test_orchestrator_node = Node(
        package='adaptive_pure_pursuit',
        executable='test_orchestrator',
        name='test_orchestrator',
        output='screen',
        parameters=[{'test_case': test_case, 'custom_path': custom_path}]
    )

    metrics_node = Node(
        package='adaptive_pure_pursuit',
        executable='control_monitor',
        name='control_monitor',
        output='screen',
        parameters=[{
            'csv_output_path': '/tmp/controller_metrics.csv',
            'plot_output_path': '/tmp/controller_metrics.png',
            'publish_rate_hz': 50.0,
            'overshoot_threshold_m': 0.2,
            'velocity_limit_m_s': 1.0,
            'angular_velocity_limit_rad_s': 1.5,
            'path_topic': '/path',
            'odom_topic': '/odometry/filtered',
            'cmd_topic': '/cmd_vel',
            'case_topic': '/testing/current_case',
        }]
    )

    rviz_visualizer_node = Node(
        package='adaptive_pure_pursuit',
        executable='rviz_visualizer',
        name='rviz_visualizer',
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file, 'r').read()}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    # 3. Create Event Handlers

    # When test_orchestrator finishes: stop the bag, then shut everything down.
    # Shutdown() sends SIGINT to all remaining nodes, including control_monitor,
    # whose own destroy_node() writes the HTML report at that point -- this is the
    # only report generated now, no external script involved.
    orchestrator_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=test_orchestrator_node,
            on_exit=[
                stop_rosbag_action,
                LogInfo(msg="======== TEST SEQUENCE FINISHED, SHUTTING DOWN ========"),
                Shutdown(),
            ],
        )
    )

    return [
        controller_node,
        test_orchestrator_node,
        metrics_node,
        rviz_visualizer_node,
        robot_state_publisher_node,
        rviz_node,
        rosbag_record,
        orchestrator_exit_handler,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'test',
            default_value='',
            description="Empty (default) = run ALL test cases. "
                         "Otherwise a single ID ('TC-03') or comma-separated list "
                         "('TC-03,TC-07,TC-10') of the cases to run."
        ),
        DeclareLaunchArgument(
            'custom_path',
            default_value='',
            description="Optional user-supplied path, e.g. '0,0;1,0;2,0.5;3,1'. "
                         "If set, this takes priority over 'test' and runs as a single "
                         "'CUSTOM' case instead of any built-in test case."
        ),
        DeclareLaunchArgument('record_bag', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])