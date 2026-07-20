#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def launch_setup(context, *args, **kwargs):
    # Retrieve launch configurations
    robot_name = LaunchConfiguration('robot_name').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    urdf_path = LaunchConfiguration('urdf_path').perform(context)
    launch_controllers = LaunchConfiguration('launch_controllers').perform(context)
    
    nodes_to_start = []
    
    # 1. Robot State Publisher (optional, if URDF/Xacro path is provided)
    if urdf_path:
        if os.path.exists(urdf_path):
            try:
                # Process xacro/urdf file
                robot_description_config = xacro.process_file(urdf_path)
                robot_description = {'robot_description': robot_description_config.toxml()}
                
                robot_state_publisher_node = Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[
                        robot_description,
                        {'use_sim_time': True}
                    ]
                )
                nodes_to_start.append(robot_state_publisher_node)
            except Exception as e:
                print(f"[spawn_robot] Error processing URDF/xacro file: {e}")
        else:
            print(f"[spawn_robot] Warning: urdf_path '{urdf_path}' does not exist.")

    # 2. Spawn Robot Entity Node
    # Reads robot description from the 'robot_description' topic
    spawn_arguments = [
        '-name', robot_name,
        '-topic', 'robot_description',
        '-x', x,
        '-y', y,
        '-z', z,
        '-Y', yaw
    ]
        
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_entity',
        arguments=spawn_arguments,
        output='screen'
    )
    nodes_to_start.append(spawn_entity)

    # 3. Robot-specific Bridges
    # Bridges key control and state topics between ROS 2 and Ignition Gazebo Fortress.
    # Scoped topics under /model/<robot_name>/ are mapped/remapped to standard ROS 2 names.
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='robot_bridge',
        arguments=[
            f'/model/{robot_name}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            f'/model/{robot_name}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            (f'/model/{robot_name}/cmd_vel', '/cmd_vel'),
            (f'/model/{robot_name}/odometry', '/odom'),
            (f'/model/{robot_name}/joint_state', '/joint_states'),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    nodes_to_start.append(bridge_node)

    # 4. Controller Spawners (for robots using ros2_control/joint trajectory/diff drive)
    if launch_controllers.lower() == 'true':
        joint_state_broadcaster_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60"
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        nodes_to_start.append(joint_state_broadcaster_spawner)

        diff_drive_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "diff_drive_controller",
                "--controller-manager-timeout", "60",
                "--switch-timeout", "60"
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
        nodes_to_start.append(diff_drive_controller_spawner)
    
    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='mars_rover', description='Name of the robot in simulation'),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y position'),
        DeclareLaunchArgument('z', default_value='0.5', description='Spawn Z position (height above terrain)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn Yaw angle'),
        DeclareLaunchArgument('urdf_path', default_value='', description='Optional absolute path to the robot URDF/xacro file to run robot_state_publisher'),
        DeclareLaunchArgument('launch_controllers', default_value='true', description='Whether to launch joint_state_broadcaster and diff_drive_controller spawners'),
        OpaqueFunction(function=launch_setup)
    ])
