import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Find the path to your package configuration
    # This replaces "$(find adaptive_pure_pursuit)"
    pkg_share = get_package_share_directory('adaptive_pure_pursuit')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    controller_node = Node(
        package='adaptive_pure_pursuit',
        executable='adaptiveLd_fixed',
        name='controller',
        output='screen',
        parameters=[config_file],
    )

    metrics_node = Node(
        package='adaptive_pure_pursuit',
        executable='metrics_evaluator',
        name='metrics_evaluator',
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
        }],
    )

    return LaunchDescription([
        controller_node,
        metrics_node,
    ])
