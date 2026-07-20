import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    obs_file_arg = DeclareLaunchArgument(
        'input_file',
        default_value='',
        description='Path to obstacle_data.npy file to visualize'
    )
    
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='marsyard',
        description='Active Gazebo world name'
    )

    spawner_node = Node(
        package='rock_generator',
        executable='spawn_rocks',
        name='rock_spawner_node',
        output='screen',
        arguments=[
            '--input', LaunchConfiguration('input_file'),
            '--world', LaunchConfiguration('world_name')
        ]
    )

    return LaunchDescription([
        obs_file_arg,
        world_name_arg,
        spawner_node
    ])
