import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the path to the params.yaml
    pkg_share = get_package_share_directory('adaptive_pure_pursuit')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # 2. Define the Controller Node
    controller_node = Node(
        package='adaptive_pure_pursuit',
        executable='adaptiveLd',    # Must match 'console_scripts' in setup.py
        name='controller',
        output='screen',
        parameters=[config_file]     # Load the YAML file
    )

    # 3. (Optional) The commented out Path Node
    # To enable this later, just uncomment the block below and add 'path_node' to the return list
    # path_node = Node(
    #     package='adaptive_pure_pursuit',
    #     executable='path_test',
    #     name='path',
    #     output='screen'
    # )

    return LaunchDescription([
        controller_node
        # , path_node
    ])