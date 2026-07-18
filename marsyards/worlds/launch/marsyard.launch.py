import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_worlds = get_package_share_directory('worlds')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
            ),
            launch_arguments={'world': 'marsyard.world'}.items()
        )
    ])
