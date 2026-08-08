#!/usr/bin/env python3
"""Publishes only the TF tree of the rover_clean rover. Nothing else runs:
no Gazebo, no controllers, no cameras, no joint_state_publisher."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_roar_rover_clean = FindPackageShare('roar_rover_clean')

    xacro_file = PathJoinSubstitution([
        pkg_roar_rover_clean, 'urdf', 'base', 'rover_simulation_clean.urdf.xacro'
    ])
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    tf_tree_publisher = Node(
        package='roar_rover_tf_publisher',
        executable='rover_tf_tree_publisher',
        name='rover_tf_tree_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_rate': 30.0,
        }],
    )

    return LaunchDescription([tf_tree_publisher])
