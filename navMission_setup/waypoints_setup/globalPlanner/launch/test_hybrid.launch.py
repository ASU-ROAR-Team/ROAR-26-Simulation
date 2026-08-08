"""Launch file for Phase 3: Hybrid Flat Terrain testing."""

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

    scenario_cfg  = LaunchConfiguration('scenario')
    start_x_cfg   = LaunchConfiguration('start_x')
    start_y_cfg   = LaunchConfiguration('start_y')
    goal_x_cfg    = LaunchConfiguration('goal_x')
    goal_y_cfg    = LaunchConfiguration('goal_y')
    new_goal_x_cfg = LaunchConfiguration('new_goal_x')
    new_goal_y_cfg = LaunchConfiguration('new_goal_y')

    return LaunchDescription([
        DeclareLaunchArgument('scenario',    default_value='corridor_block',
                              description='Hybrid scenario: corridor_block, goal_change_dynamic'),
        DeclareLaunchArgument('start_x',     default_value='0.5'),
        DeclareLaunchArgument('start_y',     default_value='0.5'),
        DeclareLaunchArgument('goal_x',      default_value='4.5'),
        DeclareLaunchArgument('goal_y',      default_value='2.5'),
        DeclareLaunchArgument('new_goal_x',  default_value='4.5'),
        DeclareLaunchArgument('new_goal_y',  default_value='0.5'),

        # D* Lite planner
        Node(
            package='dstar_navigation',
            executable='dstar_node',
            name='dstar_global_planner',
            parameters=[params_file],
            output='screen',
        ),

        # Dynamic map publisher (supports hybrid scenarios)
        Node(
            package='dstar_navigation',
            executable='dynamic_map_publisher.py',
            name='dynamic_map_publisher',
            parameters=[{
                'scenario':   scenario_cfg,
                'start_x':    start_x_cfg,
                'start_y':    start_y_cfg,
                'new_goal_x': new_goal_x_cfg,
                'new_goal_y': new_goal_y_cfg,
            }],
            output='screen',
        ),

        # Path simulator
        Node(
            package='dstar_navigation',
            executable='path_simulator.py',
            name='path_simulator',
            parameters=[{
                'start_x': start_x_cfg,
                'start_y': start_y_cfg,
            }],
            output='screen',
        ),

        # Static TF: map -> odom
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

        # Hybrid test harness
        Node(
            package='dstar_navigation',
            executable='test_harness_hybrid.py',
            name='test_harness_hybrid',
            parameters=[{
                'scenario':   scenario_cfg,
                'start_x':    start_x_cfg,
                'start_y':    start_y_cfg,
                'goal_x':     goal_x_cfg,
                'goal_y':     goal_y_cfg,
                'new_goal_x': new_goal_x_cfg,
                'new_goal_y': new_goal_y_cfg,
            }],
            on_exit=Shutdown(),
            output='screen',
        ),
    ])
