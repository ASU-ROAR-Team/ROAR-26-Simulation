"""Launch file for Phase 1 testing of D* Lite global planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('dstar_navigation')

    params_file = os.path.join(pkg_share, 'config', 'dstar_params.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz', 'dstar_test.rviz')

    dstar_node = Node(
        package='dstar_navigation',
        executable='dstar_node',
        name='dstar_global_planner',
        parameters=[params_file],
        output='screen',
    )

    map_publisher_node = Node(
        package='dstar_navigation',
        executable='map_publisher.py',
        name='map_publisher',
        output='screen',
    )

    path_simulator_node = Node(
        package='dstar_navigation',
        executable='path_simulator.py',
        name='path_simulator',
        output='screen',
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        dstar_node,
        map_publisher_node,
        path_simulator_node,
        static_tf_node,
        rviz_node,
    ])
