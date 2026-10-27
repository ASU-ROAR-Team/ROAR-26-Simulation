import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    pkg = get_package_share_directory('erc_panel_sim')

    # process xacro → URDF string
    xacro_file = os.path.join(pkg, 'urdf', 'panel.urdf.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([

        # 1. Ignition Gazebo — empty world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '--verbose', '0', '-r', 'empty.sdf'],
            output='screen'
        ),

        # 2. robot_state_publisher — publishes /tf from URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        # 3. joint_state_publisher_gui — manual joint sliders
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),

        # 4. spawn panel into Ignition using ros_gz_sim
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-name', 'erc_panel',
                '-topic', '/robot_description',
                '-x', '1.0',
                '-y', '0.0',
                '-z', '0.6'
            ]
        ),

        # 5. ros_gz_bridge — bridges /clock so use_sim_time works
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ]
        ),

        # 6. RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),

    ])
