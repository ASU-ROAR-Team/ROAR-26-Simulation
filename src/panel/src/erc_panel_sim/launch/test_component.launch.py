import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg = get_package_share_directory('erc_panel_sim')
    plate_pkg = get_package_share_directory('plate_description')
    panel_urdf_path = os.path.join(pkg, 'urdf', 'panel.urdf.xacro')
    plate_urdf_path = os.path.join(plate_pkg, 'urdf', 'plate.xacro')
    panel_desc = xacro.process_file(panel_urdf_path).toxml()
    plate_desc = xacro.process_file(plate_urdf_path).toxml()

    return LaunchDescription([
        # 1. Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-r empty.sdf'}.items()
        ),
        # 2. Panel State Publisher — RViz uses this
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': panel_desc}],
            output='screen'
        ),
        # 3. Plate State Publisher — Gazebo spawn uses this
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='plate_state_publisher',
            remappings=[('robot_description', 'plate_robot_description')],
            parameters=[{'robot_description': plate_desc}],
            output='screen'
        ),
        # 4. Joint State Publisher
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        # 5. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        ),
        # 6. Spawn Panel
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'panel',
                '-z', '0.5'
            ],
            output='screen'
        ),
        # 7. Spawn Plate
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'plate_robot_description',
                '-name', 'free_plate',
                '-x', '0.055',
                '-y', '-0.15',
                '-z', '0.8',
                '-R', '0',
                '-P', '0.5236',
                '-Y', '0'
            ],
            output='screen'
        )
    ])