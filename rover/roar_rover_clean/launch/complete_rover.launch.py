#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_roar_rover_clean = FindPackageShare('roar_rover_clean')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='X position')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Y position')
    z_arg = DeclareLaunchArgument('z', default_value='0.5', description='Z position')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='-1.5708', description='Yaw angle (-90 degrees)')
    
    urdf_file = PathJoinSubstitution([
        pkg_roar_rover_clean,
        'urdf', 'roar_complete_sim.urdf.xacro'
    ])
    
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={'world': ''}.items()
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_urdf',
        output='screen',
        arguments=[
            '-entity', 'roar_complete',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw')
        ]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    teleop_node = Node(
        package='roar_rover_clean',
        executable='teleop.py',
        name='teleop_node',
        output='screen'
    )
    
    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        joint_state_publisher_node,
        teleop_node,
    ])
