"""Standalone bringup for the D* Lite global planner.

This launch file brings up all components needed to run the D* Lite global
planner interactively, without any automated test harness.  It is intended
for manual experimentation, integration testing with an external controller,
or as a building block for a full-stack bringup.

Nodes launched
--------------
- dstar_global_planner   : D* Lite path planner (publishes /global_plan)
- dynamic_map_publisher  : Publishes an occupancy grid for the chosen scenario
- path_simulator         : Advances the robot pose along /global_plan at 0.5 m/s
- map_to_odom_tf         : Static transform  map → odom  (identity)
- rviz2                  : Visualisation with dstar_test.rviz

Launch arguments
----------------
map_scenario : Which map scenario to load (default: 'open').
               Passed directly to dynamic_map_publisher.py.
start_x      : Robot start X coordinate in metres (default: 0.5).
start_y      : Robot start Y coordinate in metres (default: 0.5).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')

    # Resolved file paths
    params_file = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')

    # Launch configurations
    map_scenario_cfg = LaunchConfiguration('map_scenario')
    start_x_cfg = LaunchConfiguration('start_x')
    start_y_cfg = LaunchConfiguration('start_y')

    # Declare launch arguments
    declare_map_scenario = DeclareLaunchArgument(
        'map_scenario',
        default_value='open',
        description='Map scenario for dynamic_map_publisher.py '
                    '(e.g. open, wall, u_trap, corridor, enclosed).',
    )
    declare_start_x = DeclareLaunchArgument(
        'start_x',
        default_value='0.5',
        description='Robot start pose X coordinate (metres).',
    )
    declare_start_y = DeclareLaunchArgument(
        'start_y',
        default_value='0.5',
        description='Robot start pose Y coordinate (metres).',
    )

    # ------------------------------------------------------------------ nodes

    # D* Lite global planner — publishes /global_plan (nav_msgs/Path)
    dstar_node = Node(
        package='dstar_navigation',
        executable='dstar_node',
        name='dstar_global_planner',
        parameters=[params_file],
        output='screen',
    )

    # Dynamic occupancy-grid publisher (mirrors the setup used in
    # test_dynamic.launch.py, but without a test harness)
    map_pub_node = Node(
        package='dstar_navigation',
        executable='dynamic_map_publisher.py',
        name='dynamic_map_publisher',
        parameters=[{
            'scenario': map_scenario_cfg,
            'start_x':  start_x_cfg,
            'start_y':  start_y_cfg,
        }],
        output='screen',
    )

    # Path simulator — moves the robot pose along the published plan at 0.5 m/s
    path_sim_node = Node(
        package='dstar_navigation',
        executable='path_simulator.py',
        name='path_simulator',
        parameters=[{
            'start_x':        start_x_cfg,
            'start_y':        start_y_cfg,
            'linear_speed':   0.5,
        }],
        output='screen',
    )

    # Static map → odom transform (identity)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # RViz2 for visualisation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        declare_map_scenario,
        declare_start_x,
        declare_start_y,
        dstar_node,
        map_pub_node,
        path_sim_node,
        static_tf_node,
        rviz_node,
    ])
