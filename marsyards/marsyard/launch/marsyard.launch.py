from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path


def _make_runtime_world(context, *args, **kwargs):
    pkg = FindPackageShare('marsyard').find('marsyard')
    models_path = os.path.join(pkg, 'models')
    model_sdf = os.path.join(models_path, 'mars_yard', 'model.sdf')

    # Build a runtime world with an absolute file:// model.sdf include.
    # This avoids Ignition resource-path guessing issues.
    runtime_world = '/tmp/marsyard_runtime.world'
    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <world name="marsyard">
    <gravity>0 0 -9.81</gravity>

    <physics name="dart_physics" type="ignored">
      <max_step_size>0.003</max_step_size>
      <real_time_update_rate>333</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact"/>

    <scene>
      <ambient>0.95 0.95 0.95 1</ambient>
      <background>0.70 0.70 0.70 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.15 0.15 0.15 1</specular>
      <direction>-0.35 0.1 -0.93</direction>
    </light>

    <include>
      <uri>file://{model_sdf}</uri>
      <name>mars_yard</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""
    Path(runtime_world).write_text(world_xml)

    resource_paths = [models_path]
    try:
        roar_sim_share = os.path.dirname(get_package_share_directory('roar_simulation'))
        resource_paths.append(roar_sim_share)
    except Exception as e:
        print(f"[marsyard.launch] Could not find roar_simulation package path: {e}")

    # Find rock_generator package if available
    try:
        pkg_rock_gen = get_package_share_directory('rock_generator')
        # rock models are under rock_generator/rocks_ws
        resource_paths.append(os.path.join(pkg_rock_gen, 'rocks_ws'))
        # Add parent to resolve package://rock_generator/rocks_ws/ paths
        resource_paths.append(os.path.dirname(pkg_rock_gen))
    except Exception as e:
        print(f"[marsyard.launch] Could not find rock_generator package: {e}")



    ign_existing = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    gz_existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    ign_path = os.pathsep.join(resource_paths)
    if ign_existing:
        ign_path = ign_path + os.pathsep + ign_existing
        
    gz_path = os.pathsep.join(resource_paths)
    if gz_existing:
        gz_path = gz_path + os.pathsep + gz_existing

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

    return [
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_path),
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value=system_plugin_path),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=system_plugin_path),
        LogInfo(msg=f'Mars Yard runtime world: {runtime_world}'),
        LogInfo(msg=f'Mars Yard model.sdf: {model_sdf}'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-v', '4', '-r', runtime_world], output='screen'),
        clock_bridge
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_make_runtime_world)
    ])
