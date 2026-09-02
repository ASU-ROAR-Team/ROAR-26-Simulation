#!/usr/bin/env python3
"""Publishes only the TF tree of the rover. Nothing else runs: no Gazebo, no
controllers, no cameras, no joint_state_publisher, no xacro/URDF needed."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    tf_tree_publisher = Node(
        package='roar_rover_tf_publisher',
        executable='rover_tf_tree_publisher',
        name='rover_tf_tree_publisher',
        output='screen',
        parameters=[{
            'publish_rate': 30.0,
        }],
    )

    return LaunchDescription([tf_tree_publisher])
