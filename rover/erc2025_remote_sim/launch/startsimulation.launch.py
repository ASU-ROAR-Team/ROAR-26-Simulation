#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
import yaml
import os

def generate_launch_description():

    gz_gui = LaunchConfiguration("gz_gui")
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value=PathJoinSubstitution(
            [FindPackageShare("erc2025_remote_sim"), "config", "teleop.config"]
        ),
        description="Run simulation with specific GUI layout.",
    )

    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="False",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution([
            FindPackageShare("erc2025_remote_sim"), "config", "components.yaml"
        ]),
        description="Path to the components configuration file for robot simulation",
    )

    declare_start_location_arg = DeclareLaunchArgument(
        "start_location",
       default_value="1",
        description="Start location ID (1, 2, 3, 4)",
       choices=["1", "2", "3", "4"],
    )
    
    start_location = LaunchConfiguration("start_location")
    package_share_directory = FindPackageShare("erc2025_remote_sim").find("erc2025_remote_sim")
    locations_file_path = os.path.join(package_share_directory, "config", "start_locations.yaml")
    with open(locations_file_path, 'r') as file:
        locations = yaml.safe_load(file)["locations"]
    location_params = locations.get(start_location, locations[1])
    
    x, y, z = location_params["position"]
    R, P, Y = location_params["orientation"]

    declare_x_arg = DeclareLaunchArgument("x", default_value=str(x), description="X position")
    declare_y_arg = DeclareLaunchArgument("y", default_value=str(y), description="Y position")
    declare_z_arg = DeclareLaunchArgument("z", default_value=str(z), description="Z position")
    declare_roll_arg = DeclareLaunchArgument("roll", default_value=str(R), description="Roll orientation")
    declare_pitch_arg = DeclareLaunchArgument("pitch", default_value=str(P), description="Pitch orientation")
    declare_yaw_arg = DeclareLaunchArgument("yaw", default_value=str(Y), description="Yaw orientation")


    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("erc2025_remote_sim"), "launch", "sim_world.launch.py"]
            )
        ),
        launch_arguments={
            "gz_gui": gz_gui,
            "gz_log_level": "1"
        }.items(),
    )

    gz_bridge_config = PathJoinSubstitution(
        [FindPackageShare("husarion_ugv_gazebo"), "config", "gz_bridge.yaml"]
    )
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": gz_bridge_config}],
    )

    simulate_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("husarion_ugv_gazebo"), "launch", "simulate_robot.launch.py"])
        ),
        launch_arguments={
            "log_level": LaunchConfiguration("log_level"),
            "components_config_path": LaunchConfiguration("components_config_path"),
        }.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("husarion_ugv_description"), "launch", "rviz.launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz")
        }.items(),
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/front_cam/zed_node/rgb/image_rect_color",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': True},
            {'front_cam.zed_node.rgb.image_rect_color.compressed.jpeg_quality': 75},
        ],
    )

    actions = [
        declare_gz_gui,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_rviz_arg,
        declare_components_config_path_arg,
        declare_start_location_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_roll_arg,
        declare_pitch_arg,
        declare_yaw_arg,
        SetUseSimTime(True),
        gz_sim,
        gz_bridge,
        simulate_robot,
        rviz_launch,
        gz_image_bridge_node
    ]

    return LaunchDescription(actions)
