import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_worlds = get_package_share_directory('worlds')
    
    world_arg = DeclareLaunchArgument(
        'world_name',
        default_value='marsyard.world',
        description='Name of the world file in the worlds package to launch'
    )
    
    marsyard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world_name')}.items()
    )
    
    return LaunchDescription([
        world_arg,
        marsyard_launch
    ])
