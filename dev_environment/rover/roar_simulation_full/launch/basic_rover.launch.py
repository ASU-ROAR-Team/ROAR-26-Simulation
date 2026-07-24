import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler,
    TimerAction
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_roar_simulation_full = get_package_share_directory('roar_simulation_full')

    # ── 1. Launch Arguments ───────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz', default_value='false',
        description='Start RViz after controllers are ready')
    default_world = os.path.join(pkg_roar_simulation_full, 'worlds', 'rover_world.sdf')
    world_file_arg = DeclareLaunchArgument(
        'world_file', default_value=default_world,
        description='Path to the SDF world file to load')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz   = LaunchConfiguration('start_rviz')
    world_file   = LaunchConfiguration('world_file')

    # ── 2. Resource paths ─────────────────────────────────────────────────────
    install_dir      = os.path.abspath(os.path.join(pkg_roar_simulation_full, '..'))
    gz_resource_path = install_dir
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_resource_path = os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir

    # ── 3. URDF / robot_description ──────────────────────────────────────────
    xacro_file = os.path.join(
        pkg_roar_simulation_full, 'urdf', 'base', 'rover_simulation.urdf.xacro')
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', xacro_file
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # ── 4. World file (now comes from launch argument) ─────────────────────

    # ── 5. Gazebo SERVER (headless) ───────────────────────────────────────────
    # The server runs physics + sensors in its own process.
    # LIBGL_ALWAYS_SOFTWARE=1 is scoped ONLY to this process so the Ogre
    # sensor render thread can create GL vertex buffers from a background
    # thread without needing a hardware EGL/GLX shared context.
    # The GUI client (below) is a separate process and uses your real AMD GPU.
    server_env = {
        'LIBGL_ALWAYS_SOFTWARE': '1',
        'IGN_GAZEBO_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_RESOURCE_PATH':     gz_resource_path,
        # Required so Ignition's plugin loader finds libign_ros2_control-system.so
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib',
    }
    gazebo_server = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_file],
        additional_env=server_env,
        output='screen'
    )

    # ── 6. Gazebo GUI CLIENT (hardware GPU) ───────────────────────────────────
    # Launched 3 s after the server so the server is ready to accept a GUI
    # connection. Uses your real AMD/Intel GPU — no software rendering here.
    client_env = {
        'QT_QPA_PLATFORM':          'xcb',
        'IGN_GAZEBO_RESOURCE_PATH': gz_resource_path,
        'GZ_SIM_RESOURCE_PATH':     gz_resource_path,
    }
    gazebo_gui = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['ign', 'gazebo', '-g'],
            additional_env=client_env,
            output='screen'
        )]
    )

    # ── 7. Robot State Publisher ──────────────────────────────────────────────
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time, 'publish_frequency': 50.0}]
    )

    # ── 8. Spawn Robot ────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'roar_rover', '-z', '2.5', '-Y', '1.5707'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 8.1 Spawn Sun Marker ──────────────────────────────────────────────────
    sun_marker_file = os.path.join(pkg_roar_simulation_full, 'worlds', 'sun_marker.sdf')
    spawn_sun = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sun_marker_file,
                   '-name', 'sun_marker', 
                   '-x', '20.0', '-y', '20.0', '-z', '15.0'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 9. ROS↔Gazebo bridge ─────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/bno055/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/bno055/mag@sensor_msgs/msg/MagneticField[ignition.msgs.Magnetometer',
            '/zed2i/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2i/depth@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/zed2i/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/world/rover_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 10. Controllers ───────────────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    diff_drive_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['diff_drive_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ── 11. RViz (optional, delayed) ─────────────────────────────────────────
    rviz_config_file = os.path.join(pkg_roar_simulation_full, 'rviz', 'basic_rover.rviz')
    rviz_args = ['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    node_rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        output='screen', arguments=rviz_args,
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'QT_QPA_PLATFORM': 'xcb'},
        condition=IfCondition(start_rviz),
    )
    # ── 12. Custom Noise & Telemetry Nodes ────────────────────────────────────
    node_zed_noise = Node(
        package='roar_simulation_full',
        executable='zed_degradation_node.py',
        name='zed_degradation_node',
        output='screen'
    )
    
    node_encoder_sim = Node(
        package='roar_simulation_full',
        executable='encoder_sim_node.py',
        name='encoder_sim_node',
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        world_file_arg,

        # Gazebo server (headless, software GL for sensors)
        gazebo_server,
        # Gazebo GUI (hardware GPU, delayed 3 s to let server start)
        gazebo_gui,

        node_robot_state_publisher,
        bridge,
        spawn_entity,
        spawn_sun,
        node_encoder_sim,
        node_zed_noise,

        # Wait for spawn → wait 12 s → start joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[TimerAction(
                    period=12.0,
                    actions=[joint_state_broadcaster_spawner]
                )]
            )
        ),

        # Joint broadcaster done → start diff_drive controller
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_controller_spawner]
            )
        ),

        # Diff drive done → start arm controller (activates arm hardware, fixes arm TF in RViz)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=diff_drive_controller_spawner,
                on_exit=[arm_controller_spawner]
            )
        ),

        # Arm controller done → start RViz (after 20 s cooldown)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[TimerAction(period=20.0, actions=[node_rviz])]
            )
        ),
    ])