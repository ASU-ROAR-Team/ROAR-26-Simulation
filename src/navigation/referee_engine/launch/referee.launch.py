import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    mission_val = LaunchConfiguration('mission').perform(context)
    
    pkg_share = get_package_share_directory('referee_engine')
    mission_directory_path = os.path.join(pkg_share, 'inputs')

    referee_node = Node(
        package='referee_engine',
        executable='referee_core_node',
        name='referee_core_node',
        output='screen',
        parameters=[{
            'mission_directory': mission_directory_path,
            'mission_name': mission_val,
            'bounds': [-1000.0, 1000.0, -1000.0, 1000.0]
        }]
    )

    return [referee_node]

def generate_launch_description():
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='mission00',
        description='Mission identifier (e.g., mission00, mission01)'
    )

    return LaunchDescription([
        mission_arg,
        OpaqueFunction(function=launch_setup)
    ])