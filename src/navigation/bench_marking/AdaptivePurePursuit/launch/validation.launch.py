import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('adaptive_pure_pursuit')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'validation_config.rviz')

    return LaunchDescription([
        # 1. Path Generator (Global and Local paths)
        Node(
            package='adaptive_pure_pursuit',
            executable='path_comprehensive',
            name='path_gen',
            output='screen'
        ),

        # 2. Controller Node (Stable parameters)
        Node(
            package='adaptive_pure_pursuit',
            executable='adaptiveLd_fixed',
            name='controller',
            parameters=[{
                'simulate_pose': True,
                'meta_parameters/log': True,
                'algorithm_parameters/KC': 0.1,
                'algorithm_parameters/KL': 0.3,
                'robot_parameters/maxVelocity': 0.5, # Slower for better observation
                'robot_parameters/min_turn_radius': 1.0,
                'robot_parameters/min_turn_speed_ratio': 0.4
            }],
            output='screen'
        ),

        # 3. Complete Validator (Diagnostic mode)
        Node(
            package='adaptive_pure_pursuit',
            executable='validator_complete',
            name='validator',
            output='screen'
        ),

        # 4. RViz Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),

        # 5. Static TF for visualization
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        )
    ])
