import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('roar_perception_testing'),
        'config', 'perception_benchmark.yaml')

    config = LaunchConfiguration('config')
    gt = LaunchConfiguration('ground_truth_path')
    aruco_gt = LaunchConfiguration('aruco_ground_truth_path')
    target = LaunchConfiguration('target_frame')
    output = LaunchConfiguration('output_directory')
    world_name = LaunchConfiguration('world_name')
    obstacle_topic = LaunchConfiguration('perception_topic')
    aruco_topic = LaunchConfiguration('aruco_topic')
    match_distance = LaunchConfiguration('match_distance_m')
    min_confirmations = LaunchConfiguration('min_confirmations')
    gt_box_padding = LaunchConfiguration('gt_box_padding_m')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'ground_truth_path',
            default_value=os.path.expanduser(
                '~/new_sim_tes/ROAR-26-Simulation/dev_environment/worlds/'
                'worldData_125/obstacle_data.npy')),
        DeclareLaunchArgument(
            'aruco_ground_truth_path',
            default_value=os.path.expanduser(
                '~/ROAR-26-Simulation/dev_environment/'
                'worlds/world/aruco_data.yaml')),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('world_name', default_value='worldData_126'),
        DeclareLaunchArgument('output_directory', default_value=''),
        DeclareLaunchArgument('perception_topic', default_value='/detection/lidar_clusters'),
        DeclareLaunchArgument('aruco_topic', default_value='/detection/aruco_3d'),
        DeclareLaunchArgument('match_distance_m', default_value='1.0'),
        DeclareLaunchArgument('min_confirmations', default_value='3'),
        DeclareLaunchArgument('gt_box_padding_m', default_value='0.45'),

        Node(
            package='roar_perception_testing',
            executable='perception_reader_node',
            name='perception_reader_node',
            output='screen',
            parameters=[config, {
                'obstacle_input_topic': obstacle_topic,
                'aruco_input_topic': aruco_topic,
            }],
        ),
        Node(
            package='roar_perception_testing',
            executable='perception_tf_node',
            name='perception_tf_node',
            output='screen',
            parameters=[config, {'target_frame': target}],
        ),
        Node(
            package='roar_perception_testing',
            executable='perception_benchmark_node',
            name='perception_benchmark_node',
            output='screen',
            parameters=[config, {
                'ground_truth_path': gt,
                'aruco_ground_truth_path': aruco_gt,
                'output_directory': output,
                'world_name': world_name,
                'target_frame': target,
                'match_distance_m': match_distance,
                'min_confirmations': min_confirmations,
                'gt_box_padding_m': gt_box_padding,
            }],
        ),
    ])
