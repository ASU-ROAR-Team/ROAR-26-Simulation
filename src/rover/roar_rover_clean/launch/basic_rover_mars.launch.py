import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_roar_rover_clean = get_package_share_directory('roar_rover_clean')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_erc2025_remote_sim = get_package_share_directory('erc2025_remote_sim')

    # 1. Declare Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2. SETUP PATHS
    install_dir = os.path.abspath(os.path.join(pkg_roar_rover_clean, '..'))
    marsyard_share_dir = os.path.abspath(pkg_erc2025_remote_sim)
    world_file = os.path.join(marsyard_share_dir, 'worlds', 'marsyard2024.world')

    resource_paths = [install_dir, marsyard_share_dir]
    if 'GZ_SIM_RESOURCE_PATH' in os.environ and os.environ['GZ_SIM_RESOURCE_PATH']:
        resource_paths.insert(0, os.environ['GZ_SIM_RESOURCE_PATH'])
    gz_resource_path = ':'.join(resource_paths)

    ign_resource_paths = [install_dir, marsyard_share_dir]
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ and os.environ['IGN_GAZEBO_RESOURCE_PATH']:
        ign_resource_paths.insert(0, os.environ['IGN_GAZEBO_RESOURCE_PATH'])
    ign_gazebo_resource_path = ':'.join(ign_resource_paths)

    # 3. Process URDF
    xacro_file = os.path.join(pkg_roar_rover_clean, 'urdf', 'base', 'rover_simulation.urdf.xacro')
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 4. Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r {}'.format(world_file)}.items(),
    )

    # 5. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 6. Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'roar_rover', '-z', '0.5', '-Y', '1.5707'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7. Bridge Clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 8. Controllers (Removed the 30s timeout arg as the C++ core ignores it anyway)
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        
        # Set resource paths for Gazebo Sim and Ignition Gazebo compatibility
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_resource_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_gazebo_resource_path),
        
        # --- THE FIXES ---
        # 1. Force Qt to use X11 instead of Wayland to prevent the libEGL crash across different machines
        #SetEnvironmentVariable(name='QT_QPA_PLATFORM', value='xcb'),
        
        gazebo,
        bridge,
        node_robot_state_publisher,
        spawn_entity,
        
        # 2. Wait for robot URDF to be sent -> WAIT 12 SECONDS for meshes to render -> Start broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=12.0, 
                        actions=[joint_state_broadcaster_spawner]
                    )
                ],
            )
        ),
        
        # Wait for joint states -> Start drive controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner],
            )
        ),
    ])