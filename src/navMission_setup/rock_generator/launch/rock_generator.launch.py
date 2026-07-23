import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_worlds = get_package_share_directory('worlds')
    
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='marsyard.world',
        description='World filename from worlds package'
    )
    density_arg = DeclareLaunchArgument(
        'density',
        default_value='0.012',
        description='Rock density (rocks per sq. meter)'
    )
    collidable_ratio_arg = DeclareLaunchArgument(
        'collidable_ratio',
        default_value='0.5',
        description='Collidable rock ratio (0.0 to 1.0)'
    )
    spacing_arg = DeclareLaunchArgument(
        'spacing',
        default_value='1.0',
        description='Minimum spacing between rock centers in meters'
    )

    # 1. Launch World from worlds package
    launch_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name')}.items()
    )

    # 2. Generator and Spawner Node
    gen_and_spawn_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rock_generator', 'rock_generator',
            '--world-name', LaunchConfiguration('world_name'),
            '--density', LaunchConfiguration('density'),
            '--collidable-ratio', LaunchConfiguration('collidable_ratio'),
            '--spacing', LaunchConfiguration('spacing')
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name_arg,
        density_arg,
        collidable_ratio_arg,
        spacing_arg,
        launch_map,
        gen_and_spawn_process
    ])
