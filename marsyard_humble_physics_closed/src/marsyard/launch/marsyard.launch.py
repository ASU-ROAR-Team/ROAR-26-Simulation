from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('marsyard')
    world = os.path.join(pkg_share, 'worlds', 'marsyard.sdf')
    models = os.path.join(pkg_share, 'models')

    # Construct the resource paths for Gazebo to find meshes of roar_simulation
    resource_paths = [models]
    try:
        roar_sim_share = os.path.dirname(get_package_share_directory('roar_simulation'))
        resource_paths.append(roar_sim_share)
    except Exception as e:
        print(f"[marsyard.launch] Could not find roar_simulation package path: {e}")

    current_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    if current_ign:
        resource_paths.append(current_ign)
    current_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if current_gz:
        resource_paths.append(current_gz)

    resource_path = os.pathsep.join(resource_paths)

    # Construct the plugin paths for Gazebo to find libraries like gz_ros2_control
    system_plugin_paths = ['/opt/ros/humble/lib']
    current_ign_plugin = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    if current_ign_plugin:
        system_plugin_paths.append(current_ign_plugin)
    current_gz_plugin = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    if current_gz_plugin:
        system_plugin_paths.append(current_gz_plugin)
    system_plugin_path = os.pathsep.join(system_plugin_paths)

    # Bridge for simulator clock so ROS 2 nodes can sync with simulation time
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', system_plugin_path),
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', system_plugin_path),
        LogInfo(msg='Launching Mars Yard environment only: no rover, no dummy objects, Ignition Gazebo Fortress.'),
        LogInfo(msg=['World: ', world]),
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', world],
            output='screen'
        ),
        clock_bridge
    ])
