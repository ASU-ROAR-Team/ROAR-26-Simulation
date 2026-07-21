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
        description='Minimum centre-to-centre spacing between rocks in metres'
    )
    min_terrain_height_arg = DeclareLaunchArgument(
        'min_terrain_height',
        default_value='0.15',
        description='Minimum Z height (metres) for a point to be considered valid terrain.'
    )
    min_roughness_arg = DeclareLaunchArgument(
        'min_roughness',
        default_value='0.02',
        description='Minimum local Z std-dev to accept a cell as rough terrain. '
                    'Increase to exclude flatter areas (e.g. 0.05), decrease for gentle slopes.'
    )
    deadends_arg = DeclareLaunchArgument(
        'deadends',
        default_value='False',
        description='Place a barrier formation of rocks across the course centre'
    )

    # 1. Launch the Gazebo world
    launch_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name')}.items()
    )

    # 2. Run the heightmap-based generator + spawner
    gen_and_spawn_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rock_generator', 'rock_generator',
            '--world-name',        LaunchConfiguration('world_name'),
            '--density',           LaunchConfiguration('density'),
            '--collidable-ratio',  LaunchConfiguration('collidable_ratio'),
            '--spacing',           LaunchConfiguration('spacing'),
            '--min-terrain-height', LaunchConfiguration('min_terrain_height'),
            '--min-roughness',      LaunchConfiguration('min_roughness'),
        ],
        output='screen'
    )

    return LaunchDescription([
        world_name_arg,
        density_arg,
        collidable_ratio_arg,
        spacing_arg,
        min_terrain_height_arg,
        min_roughness_arg,
        deadends_arg,
        launch_map,
        gen_and_spawn_process,
    ])
