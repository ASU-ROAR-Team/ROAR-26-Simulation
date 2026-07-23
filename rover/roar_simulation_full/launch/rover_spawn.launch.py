#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    pkg_roar_simulation_full = FindPackageShare('roar_simulation_full')
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    
    # Get the install directory for mesh files
    pkg_share = os.path.join(os.path.expanduser('~'), 'roar_workspace', 'install', 'roar_simulation_full', 'share')
    
    # Set Ignition resource path
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=pkg_share
    )
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=pkg_share
    )
    
    # URDF path and processing
    urdf_file = PathJoinSubstitution([pkg_roar_simulation_full, 'urdf', 'base', 'rover_simulation.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
    
    # 1. Ignition Gazebo World
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )
    
    # 3. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'roar_rover', '-topic', 'robot_description'],
        output='screen'
    )

    # 4. Bridge (Clock and Basic Topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )
    
    return LaunchDescription([
        set_ign_resource_path,
        set_gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])
