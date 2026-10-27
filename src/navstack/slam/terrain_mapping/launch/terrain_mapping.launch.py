import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Default to world1's heightmap in slam/worlds — override with a launch
    # argument to point at a different world's heightmap.npz.
    try:
        default_heightmap_file = os.path.join(
            get_package_share_directory("terrain_mapping"), "working_heightmaps", "heightmap_mars_yard.npz"
        )
    except PackageNotFoundError:
        default_heightmap_file = ""

    # Default to roar_localization's ArUco marker map, the same file used for
    # its landmark-correction localization, so mapping and localization never
    # disagree on where the markers are. Falls back to "" (node no-ops) if
    # roar_localization isn't built in this workspace.
    try:
        default_aruco_map_file = os.path.join(
            get_package_share_directory("roar_localization"), "config", "aruco_map.yaml"
        )
    except PackageNotFoundError:
        default_aruco_map_file = ""

    heightmap_file_arg = DeclareLaunchArgument(
        "heightmap_file",
        default_value=default_heightmap_file,
        description="Absolute path to the static .npz height map to load",
    )
    aruco_map_file_arg = DeclareLaunchArgument(
        "aruco_map_file",
        default_value=default_aruco_map_file,
        description="Absolute path to the ArUco marker world-position YAML to stamp as static obstacles",
    )
    aruco_obstacle_height_arg = DeclareLaunchArgument(
        "aruco_obstacle_height",
        default_value="1.5",
        description="Height [m] above terrain to stamp each ArUco marker obstacle",
    )

    node = Node(
        package="terrain_mapping",
        executable="terrain_mapping_node",
        name="terrain_mapping",
        output="screen",
        parameters=[
            {
                "heightmap_file": LaunchConfiguration("heightmap_file"),
                "aruco_map_file": LaunchConfiguration("aruco_map_file"),
                "aruco_obstacle_height": LaunchConfiguration("aruco_obstacle_height"),
            },
        ],
    )

    return LaunchDescription([
        heightmap_file_arg,
        aruco_map_file_arg,
        aruco_obstacle_height_arg,
        node,
    ])
