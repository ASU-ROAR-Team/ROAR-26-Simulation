import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Sibling directory of this repo's own checkout
_repo_root = Path(__file__).resolve().parents[2]
_sibling_root = _repo_root.parent


def _resolve_sim_ws():
    override = os.environ.get('ROAR_SIM_WS')
    if override and os.path.exists(override):
        return os.path.expanduser(override)
    current = Path(__file__).resolve()
    for parent in [current] + list(current.parents):
        cand = parent / "src" / "simulation"
        if cand.is_dir():
            return str(cand)
    return str(_sibling_root / "ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion")


def launch_setup(context, *args, **kwargs):
    # ── Resolve launch arg values ───────────────────────────────────────────
    world_num_val = LaunchConfiguration('world_num').perform(context)
    rover_module_val = LaunchConfiguration('rover_module').perform(context)
    mission_val = LaunchConfiguration('mission').perform(context)
    headless_val = LaunchConfiguration('headless').perform(context)
    rviz_val = LaunchConfiguration('rviz').perform(context)
    enable_slam_val = LaunchConfiguration('enable_slam').perform(context)
    enable_perception_val = LaunchConfiguration('enable_perception').perform(context)
    enable_pursuit_val = LaunchConfiguration('enable_pursuit').perform(context)

    # ── Paths ────────────────────────────────────────────────────────────────
    try:
        sim_pkg_share = get_package_share_directory(rover_module_val)
    except Exception as e:
        return [LogInfo(msg=f"[ERROR] Could not find rover module package: {rover_module_val}")]

    sim_launch_file = os.path.join(sim_pkg_share, 'launch', 'basic_rover.launch.py')

    roar_26_ws = _resolve_sim_ws()
    world_file = os.path.join(roar_26_ws, 'dev_environment', 'worlds', f'world{world_num_val}', f'world{world_num_val}.world')

    if not os.path.exists(world_file):
        fallback_world = os.path.join(roar_26_ws, 'navMission_setup', 'outputs', f'world{world_num_val}', f'world{world_num_val}.world')
        if os.path.exists(fallback_world):
            world_file = fallback_world
        else:
            return [LogInfo(msg=f"[ERROR] Could not find world file for world_num={world_num_val}")]

    # NavStack launch file
    nav_launch_file = None
    try:
        nav_pkg_share = get_package_share_directory('navstack_nav2')
        nav_launch_file = os.path.join(nav_pkg_share, 'launch', 'roar_nav2_only.launch.py')
    except Exception as e:
        pass

    # Referee/Evaluation launch file
    referee_pkg_share = get_package_share_directory('referee_engine')
    eval_launch_file = os.path.join(referee_pkg_share, 'launch', 'roar_full_evaluation.launch.py')

    nodes_and_launches = []

    nodes_and_launches.append(LogInfo(msg=f"============================================================"))
    nodes_and_launches.append(LogInfo(msg=f"MASTER LAUNCH: Starting Full Evaluation Pipeline"))
    nodes_and_launches.append(LogInfo(msg=f"  Rover Module : {rover_module_val}"))
    nodes_and_launches.append(LogInfo(msg=f"  World Number : {world_num_val}"))
    nodes_and_launches.append(LogInfo(msg=f"  World File   : {world_file}"))
    nodes_and_launches.append(LogInfo(msg=f"  Mission      : {mission_val}"))
    nodes_and_launches.append(LogInfo(msg=f"============================================================"))

    # 1. Launch Simulation
    nodes_and_launches.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_file),
            launch_arguments={
                'use_sim_time': 'true',
                'world_file': world_file,
                'headless': headless_val
            }.items()
        )
    )

    # 2 & 3. Delayed Launch for Nav Stack and Referee Engine
    delayed_actions = []

    if nav_launch_file and os.path.exists(nav_launch_file):
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_file),
            launch_arguments={'rviz': rviz_val, 'enable_slam': enable_slam_val}.items()
        )
        delayed_actions.append(nav_launch)
    else:
        nodes_and_launches.append(LogInfo(msg="[WARNING] Skipping Nav Stack launch (file not found)."))

    eval_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(eval_launch_file),
        launch_arguments={
            'mission': mission_val,
            'world_num': world_num_val,
            'enable_slam': 'false',  # Disabled because Nav Stack launches it
            'enable_perception': enable_perception_val,
            'enable_pursuit': enable_pursuit_val
        }.items()
    )
    delayed_actions.append(eval_launch)

    # Launch them both together after joint_states is available
    wait_for_sim = ExecuteProcess(
        cmd=['ros2', 'topic', 'echo', '--once', '/joint_states'],
        output='log'
    )
    nodes_and_launches.append(wait_for_sim)

    nodes_and_launches.append(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_sim,
                on_exit=delayed_actions
            )
        )
    )

    return nodes_and_launches

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_num',
            default_value='1',
            description='Selected benchmark world number: 1, 2, or 3'
        ),
        DeclareLaunchArgument(
            'rover_module',
            default_value='roar_simulation_full',
            description='Rover package to launch (e.g. roar_simulation_full, roar_rover_noise, roar_rover_clean)'
        ),
        DeclareLaunchArgument(
            'mission',
            default_value='mission_1',
            description='Mission folder name inside referee inputs/'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Launch Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Enable SLAM evaluator node'
        ),
        DeclareLaunchArgument(
            'enable_perception',
            default_value='false',
            description='Enable Perception evaluator node'
        ),
        DeclareLaunchArgument(
            'enable_pursuit',
            default_value='false',
            description='Enable Pure Pursuit evaluator node'
        ),
        OpaqueFunction(function=launch_setup)
    ])
