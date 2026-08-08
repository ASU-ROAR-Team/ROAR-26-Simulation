"""Launch file for Phase 2: Dynamic Obstacle Updates testing."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')
    
    # Paths
    params_file = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')
    
    # Launch Configurations
    scenario_cfg = LaunchConfiguration('scenario')
    start_x_cfg = LaunchConfiguration('start_x')
    start_y_cfg = LaunchConfiguration('start_y')
    goal_x_cfg = LaunchConfiguration('goal_x')
    goal_y_cfg = LaunchConfiguration('goal_y')
    
    # Declare Launch Arguments
    declare_scenario = DeclareLaunchArgument('scenario', default_value='known_maze_blocked', description='Dynamic scenario: known_maze_blocked, lidar_maze_120')
    declare_start_x = DeclareLaunchArgument('start_x', default_value='0.5', description='Start pose X coordinate')
    declare_start_y = DeclareLaunchArgument('start_y', default_value='2.0', description='Start pose Y coordinate')
    declare_goal_x = DeclareLaunchArgument('goal_x', default_value='4.5', description='Goal pose X coordinate')
    declare_goal_y = DeclareLaunchArgument('goal_y', default_value='2.0', description='Goal pose Y coordinate')

    # Node: D* Lite Planner
    dstar_node = Node(
        package='dstar_navigation',
        executable='dstar_node',
        name='dstar_global_planner',
        parameters=[params_file],
        output='screen',
    )

    # Node: Dynamic Map Publisher
    map_pub_node = Node(
        package='dstar_navigation',
        executable='dynamic_map_publisher.py',
        name='dynamic_map_publisher',
        parameters=[{
            'scenario': scenario_cfg,
            'start_x': start_x_cfg,
            'start_y': start_y_cfg
        }],
        output='screen',
    )

    # Node: Path Simulator
    path_sim_node = Node(
        package='dstar_navigation',
        executable='path_simulator.py',
        name='path_simulator',
        parameters=[{
            'start_x': start_x_cfg,
            'start_y': start_y_cfg
        }],
        output='screen',
    )

    # Node: Static Map to Odom TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    # Node: RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Node: Dynamic Test Harness
    harness_node = Node(
        package='dstar_navigation',
        executable='test_harness_dynamic.py',
        name='test_harness_dynamic',
        parameters=[{
            'scenario': scenario_cfg,
            'start_x': start_x_cfg,
            'start_y': start_y_cfg,
            'goal_x': goal_x_cfg,
            'goal_y': goal_y_cfg
        }],
        on_exit=Shutdown(),
        output='screen',
    )

    return LaunchDescription([
        declare_scenario,
        declare_start_x,
        declare_start_y,
        declare_goal_x,
        declare_goal_y,
        dstar_node,
        map_pub_node,
        path_sim_node,
        static_tf_node,
        rviz_node,
        harness_node
    ])
