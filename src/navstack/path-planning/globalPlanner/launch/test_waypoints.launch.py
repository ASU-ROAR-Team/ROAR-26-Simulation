"""Launch file for Multi-Waypoint Navigation testing."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')

    params_file = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')

    start_x_cfg  = LaunchConfiguration('start_x')
    start_y_cfg  = LaunchConfiguration('start_y')

    return LaunchDescription([
        DeclareLaunchArgument('start_x',  default_value='1.0'),
        DeclareLaunchArgument('start_y',  default_value='1.0'),

        # D* Lite Planner
        Node(
            package='dstar_navigation',
            executable='dstar_node',
            name='dstar_global_planner',
            parameters=[params_file],
            output='screen',
        ),

        # ERC Map Generator
        Node(
            package='dstar_navigation',
            executable='erc_map_generator.py',
            name='erc_map_generator',
            parameters=[{
                'start_x': start_x_cfg,
                'start_y': start_y_cfg,
            }],
            output='screen',
        ),

        # Path Simulator
        Node(
            package='dstar_navigation',
            executable='path_simulator.py',
            name='path_simulator',
            parameters=[{
                'start_x': start_x_cfg,
                'start_y': start_y_cfg,
                'speed': 0.5,
            }],
            output='screen',
        ),

        # Static TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),

        # Multi-Waypoint Test Harness
        Node(
            package='dstar_navigation',
            executable='test_harness_waypoints.py',
            name='test_harness_waypoints',
            parameters=[{
                'start_x': start_x_cfg,
                'start_y': start_y_cfg,
            }],
            on_exit=Shutdown(),
            output='screen',
        ),
    ])
