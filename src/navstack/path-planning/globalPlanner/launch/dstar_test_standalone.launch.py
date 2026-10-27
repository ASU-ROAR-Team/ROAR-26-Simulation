import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_dir = os.path.join(
        get_package_share_directory('dstar_navigation'),
        'config',
        'rviz',
        'dstar_test.rviz'
    )

    return LaunchDescription([
        Node(
            package='dstar_navigation',
            executable='fake_slam.py',
            name='fake_slam',
            parameters=[{'use_sim_time': False}]
        ),
        
        Node(
            package='dstar_navigation',
            executable='path_follower.py',
            name='path_follower',
            parameters=[{'use_sim_time': False}]
        ),
        
        Node(
            package='dstar_navigation',
            executable='heightmap_to_costmap.py',
            name='heightmap_to_costmap',
            parameters=[{
                'use_sim_time': False,
                'heightmap_topic': '/active_map/heightmap',
                'heightmap_range_topic': '/active_map/heightmap_range',
                'unified_costmap_topic': '/unified_costmap',
                'MAX_SAFE_SLOPE': 1.0, #45 degrees for now
                'MAX_SAFE_ROUGHNESS': 1.0,
                'steepness_weight': 0.5,
                'roughness_weight': 0.05,
                'max_traversable_height_diff': 0.35,
                'publish_rate': 2.0,
                'inscribed_radius': 0.35,
                'inflation_radius': 0.8,
                'inflation_cost_scaling': 9.0
            }]
        ),
        
        Node(
            package='dstar_navigation',
            executable='dstar_node',
            name='dstar_node',
            parameters=[{
                'use_sim_time': False,
                'lethal_cost_threshold': 253,
                'unknown_traversal_cost': 1.0,
                'allow_unknown': True,
                'heuristic_weight': 1.0,
                'enable_clearance_field': True,
                'clearance_field_weight': 1.3,
                'clearance_decay_distance': 0.3,
                'terrain_weight': 2.0,
                'enable_incremental_updates': True,
                'unified_costmap_topic': '/unified_costmap'
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': False}]
        )
    ])
