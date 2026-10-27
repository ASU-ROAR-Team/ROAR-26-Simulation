import os
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Declare Configurable Launch Arguments ---
    path_topic_arg = DeclareLaunchArgument(
        'path_topic',
        default_value='/plan',
        description='Topic name for target path/trajectory'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for robot odometry feedback'
    )

    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel',
        description='Topic name for controller velocity commands'
    )

    case_topic_arg = DeclareLaunchArgument(
        'case_topic',
        default_value='/testing/current_case',
        description='Topic name for broadcasting current test case ID'
    )

    test_case_arg = DeclareLaunchArgument(
        'test_case',
        default_value='all',
        description='Test case selection ("all" or specific case like "TC-01")'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='50.0',
        description='Evaluator sampling and publishing rate in Hz'
    )

    global_timeout_arg = DeclareLaunchArgument(
        'global_timeout_s',
        default_value='30.0',
        description='Timeout per test case in seconds'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to automatically launch RViz2'
    )

    default_csv = os.path.join(tempfile.gettempdir(), 'controller_global_behavior.csv')
    default_plot = os.path.join(tempfile.gettempdir(), 'controller_global_behavior.png')

    csv_output_arg = DeclareLaunchArgument(
        'csv_output_path',
        default_value=default_csv,
        description='Absolute path to log continuous CSV metrics'
    )

    plot_output_arg = DeclareLaunchArgument(
        'plot_output_path',
        default_value=default_plot,
        description='Absolute path to save post-run diagnostic plots'
    )

    # Physical Envelope Limits (Default matching MPPI vx_max=0.7 m/s, wz_max=1.2 rad/s)
    v_limit_arg = DeclareLaunchArgument('velocity_limit_m_s', default_value='0.7')
    w_limit_arg = DeclareLaunchArgument('angular_velocity_limit_rad_s', default_value='1.2')
    max_jerk_arg = DeclareLaunchArgument('max_allowed_jerk_deg_s2', default_value='150.0')

    controller_param_file_arg = DeclareLaunchArgument(
        'controller_param_file',
        default_value='',
        description='Optional path to controller parameter YAML file (e.g., nav2_mppi_params.yaml) to automatically extract speed limits and tolerances'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether to broadcast odom->base_link TF frame (keep false when robot/sim already publishes TF)'
    )

    # Path to executable script relative to launch file location
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script_path = os.path.join(pkg_dir, 'src', 'integrated_evaluator.py')

    # --- Evaluator Node Definition ---
    evaluator_node = Node(
        executable=script_path, 
        name='integrated_evaluator_node',
        output='screen',
        parameters=[{
            'path_topic': LaunchConfiguration('path_topic'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'case_topic': LaunchConfiguration('case_topic'),
            'test_case': LaunchConfiguration('test_case'),
            'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
            'global_timeout_s': LaunchConfiguration('global_timeout_s'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'csv_output_path': LaunchConfiguration('csv_output_path'),
            'plot_output_path': LaunchConfiguration('plot_output_path'),
            'velocity_limit_m_s': LaunchConfiguration('velocity_limit_m_s'),
            'angular_velocity_limit_rad_s': LaunchConfiguration('angular_velocity_limit_rad_s'),
            'max_allowed_jerk_deg_s2': LaunchConfiguration('max_allowed_jerk_deg_s2'),
            'controller_param_file': LaunchConfiguration('controller_param_file'),
        }]
    )

    return LaunchDescription([
        path_topic_arg,
        odom_topic_arg,
        cmd_topic_arg,
        case_topic_arg,
        test_case_arg,
        publish_rate_arg,
        global_timeout_arg,
        launch_rviz_arg,
        csv_output_arg,
        plot_output_arg,
        v_limit_arg,
        w_limit_arg,
        max_jerk_arg,
        controller_param_file_arg,
        publish_tf_arg,
        evaluator_node,
    ])