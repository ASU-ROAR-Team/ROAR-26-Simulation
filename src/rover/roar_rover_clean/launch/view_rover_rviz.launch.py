#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_roar_rover_clean = FindPackageShare('roar_rover_clean')
    
    # URDF path and processing
    urdf_file = PathJoinSubstitution([
        pkg_roar_rover_clean, 
        'urdf', 'base', 'rover_simulation.urdf.xacro'
    ])
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # Robot State Publisher (publishes robot structure)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )
    
    # Joint State Publisher GUI (sliders to move joints)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz (visualization)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
