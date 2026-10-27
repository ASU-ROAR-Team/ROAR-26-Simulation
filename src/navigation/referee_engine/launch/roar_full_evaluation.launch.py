"""
roar_full_evaluation.launch.py
==============================
One-click launch for the ASU ROAR evaluation suite (NO mock driver):

  • Referee Engine         (always ON)
  • Report Generator       (always ON)
  • Collision Monitor      (always ON)
  • SLAM Evaluator         (opt-in: enable_slam:=true)
  • Perception Benchmark   (opt-in: enable_perception:=true)
  • Adaptive Pure Pursuit  (opt-in: enable_pursuit:=true)

Usage:
  ros2 launch referee_engine roar_full_evaluation.launch.py \\
      mission:=mission_1 \\
      enable_slam:=true enable_perception:=true enable_pursuit:=true
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
# Fixed report directories
# ─────────────────────────────────────────────────────────────────────────────
HOME = os.path.expanduser('~')

def _resolve_ws_root():
    override = os.environ.get('ROAR_WS') or os.environ.get('ROAR_EVAL_WS')
    if override and os.path.exists(override):
        return os.path.abspath(os.path.expanduser(override))
    current = Path(__file__).resolve()
    for parent in current.parents:
        if ((parent / 'src' / 'navigation' / 'missions').is_dir() and
                (parent / 'src' / 'roar2026' / 'dev_environment').is_dir()):
            return str(parent)

    raise RuntimeError(
        'Could not locate the ROAR workspace. Set ROAR_WS to the workspace root.')

EVAL_WS = _resolve_ws_root()


REPORT_ROOT = os.path.join(EVAL_WS, 'reports')

REFEREE_REPORT = os.path.join(REPORT_ROOT, 'referee')
SLAM_REPORT = os.path.join(REPORT_ROOT, 'slam')
PERCEPTION_REPORT = os.path.join(REPORT_ROOT, 'perception')
PURSUIT_REPORT = os.path.join(REPORT_ROOT, 'pursuit')

for directory in (
    REFEREE_REPORT,
    SLAM_REPORT,
    PERCEPTION_REPORT,
    PURSUIT_REPORT,
):
    os.makedirs(directory, exist_ok=True)


def launch_setup(context, *args, **kwargs):
    # ── Resolve launch arg values ───────────────────────────────────────────
    mission_val      = LaunchConfiguration('mission').perform(context)
    enable_slam      = LaunchConfiguration('enable_slam').perform(context).lower()
    enable_percept   = LaunchConfiguration('enable_perception').perform(context).lower()
    enable_pursuit   = LaunchConfiguration('enable_pursuit').perform(context).lower()
    world_num        = LaunchConfiguration('world_num').perform(context)
    yaml_params_val  = LaunchConfiguration('yaml_params_file').perform(context)

    # ── Package share directories ────────────────────────────────────────────
    referee_share   = get_package_share_directory('referee_engine')
    mission_dir     = os.path.join(referee_share, 'inputs')

    nodes = []

    # ── 1. Referee Core ──────────────────────────────────────────────────────
    nodes.append(LogInfo(msg=f"[ROAR EVAL] Referee → mission={mission_val}"))
    nodes.append(Node(
        package='referee_engine',
        executable='referee_core_node',
        name='referee_core_node',
        output='screen',
        parameters=[{
            'mission_directory': mission_dir,
            'mission_name':      mission_val,
            'yaml_params_file':  yaml_params_val,
            'bounds':            [-1000.0, 1000.0, -1000.0, 1000.0],
        }],
    ))

    # ── 2. Report Generator ──────────────────────────────────────────────────
    nodes.append(Node(
        package='referee_engine',
        executable='report_generator_node',
        name='report_generator_node',
        output='screen',
        parameters=[{
            'output_dir': REFEREE_REPORT,
            'mission_name': mission_val
        }],
    ))

    # ── 3. Collision Monitor ─────────────────────────────────────────────────
    world_name = f'world{world_num}'

    # Bridge for the 7 contact topics
    contact_topics = [
        f'/world/{world_name}/model/roar_rover/link/base_link/sensor/base_link_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_rhs_front/sensor/wheel_rhs_front_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_rhs_mid/sensor/wheel_rhs_mid_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_rhs_rear/sensor/wheel_rhs_rear_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_lhs_front/sensor/wheel_lhs_front_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_lhs_mid/sensor/wheel_lhs_mid_contact/contact',
        f'/world/{world_name}/model/roar_rover/link/wheel_lhs_rear/sensor/wheel_lhs_rear_contact/contact'
    ]

    bridge_args = []
    for topic in contact_topics:
        bridge_args.append(f'{topic}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts')

    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='contact_bridge',
        output='screen',
        arguments=bridge_args
    ))

    mission_assets = os.path.join(
        EVAL_WS,
        'src',
        'navigation',
        'missions',
        f'mission_{world_num}',
        'assets'
    )
    obstacle_gt = os.path.join(
        mission_assets,
        'obstacle_data.npy'
    )

    nodes.append(Node(
        package='referee_engine',
        executable='collision_monitor_node',
        name='collision_monitor_node',
        output='screen',
        parameters=[{
            'world_name': world_name,
            'model_name': 'roar_rover',
            'obstacle_gt_path': obstacle_gt
        }],
    ))

    # ── 4. SLAM Evaluator ────────────────────────────────────────────────────
    if enable_slam in ('true', '1', 'yes'):
        nodes.append(LogInfo(msg="[ROAR EVAL] SLAM evaluator ENABLED"))

        slam_gt_topic = '/ground_truth/pose'
        slam_odom_topic = '/diff_drive_controller/odom'

        nodes.append(Node(
            package='roar_slam_benchmark',
            executable='slam_evaluator',
            name='slam_evaluator',
            output='screen',
            parameters=[{
                'output_dir': SLAM_REPORT,
                'ground_truth_topic': slam_gt_topic,
                'odom_topic': slam_odom_topic
            }],
        ))

    # ── 5. Perception Benchmark ──────────────────────────────────────────────
    if enable_percept in ('true', '1', 'yes'):
        nodes.append(LogInfo(msg="[ROAR EVAL] Perception benchmark ENABLED"))
        try:
            percept_share  = get_package_share_directory('roar_perception_testing')
            percept_config = os.path.join(
                percept_share,
                'config',
                'perception_benchmark.yaml'
            )

            aruco_gt = os.path.join(
                mission_assets,
                'aruco_data.yaml'
            )

            percept_outdir = os.path.join(
                EVAL_WS,
                'reports',
                'perception'
            )

            os.makedirs(
                percept_outdir,
                exist_ok=True
            )

            nodes.append(LogInfo(
                msg=(
                    f"[ROAR EVAL] Perception world={world_name} "
                    f"obstacle_gt={obstacle_gt} "
                    f"aruco_gt={aruco_gt}"
                )
            ))

            nodes.append(Node(
                package='roar_perception_testing',
                executable='perception_reader_node',
                name='perception_reader_node',
                output='screen',
                parameters=[percept_config],
            ))
            nodes.append(Node(
                package='roar_perception_testing',
                executable='perception_tf_node',
                name='perception_tf_node',
                output='screen',
                parameters=[percept_config],
            ))
            nodes.append(Node(
                package='roar_perception_testing',
                executable='perception_benchmark_node',
                name='perception_benchmark_node',
                output='screen',
                parameters=[
                    percept_config,
                    {
                        'output_directory': percept_outdir,
                        'world_name': world_name,
                        'ground_truth_path': obstacle_gt,
                        'aruco_ground_truth_path': aruco_gt,
                    }
                ],
            ))
        except Exception as e:
            nodes.append(LogInfo(msg=f"[ROAR EVAL] WARNING: roar_perception_testing not found: {e}"))

    # ── 6. Adaptive Pure Pursuit Controller ──────────────────────────────────
    if enable_pursuit in ('true', '1', 'yes'):
        nodes.append(LogInfo(msg="[ROAR EVAL] Adaptive Pure Pursuit ENABLED"))

        try:
            pursuit_share  = get_package_share_directory('adaptive_pure_pursuit')
            pursuit_config = os.path.join(pursuit_share, 'config', 'params.yaml')
            pursuit_report = os.path.join(EVAL_WS, 'reports', 'pursuit')
            os.makedirs(pursuit_report, exist_ok=True)

            odom_topic_name = '/odometry/filtered'

            nodes.append(Node(
                package='adaptive_pure_pursuit',
                executable='adaptiveLd_fixed',
                name='adaptive_pure_pursuit_controller',
                output='screen',
                parameters=[pursuit_config],
            ))
            nodes.append(Node(
                package='adaptive_pure_pursuit',
                executable='control_monitor',
                name='pursuit_control_monitor',
                output='screen',
                parameters=[{
                    'csv_output_path':              os.path.join(pursuit_report, 'controller_metrics.csv'),
                    'plot_output_path':             os.path.join(pursuit_report, 'controller_metrics.png'),
                    'publish_rate_hz':              50.0,
                    'overshoot_threshold_m':        0.2,
                    'velocity_limit_m_s':           1.0,
                    'angular_velocity_limit_rad_s': 1.5,
                    'path_topic':                   '/path',
                    'odom_topic':                   odom_topic_name,
                    'cmd_topic':                    '/cmd_vel',
                    'case_topic':                   '/testing/current_case',
                }],
            ))
        except Exception as e:
            nodes.append(LogInfo(msg=f"[ROAR EVAL] WARNING: adaptive_pure_pursuit not found: {e}"))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ── Mission args ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'mission',
            default_value='mission_1',
            description='Mission folder name inside referee inputs/',
        ),
        DeclareLaunchArgument(
            'world_num',
            default_value='1',
            description='Selected benchmark world number: 1, 2, or 3',
        ),
        DeclareLaunchArgument(
            'yaml_params_file',
            default_value='',
            description='Path to YAML parameter file (matching erc_waypoint_follower.py format)',
        ),
        # ── Subsystem toggles ────────────────────────────────────────────────
        DeclareLaunchArgument(
            'enable_slam',
            default_value='true',
            description='Set true to start the SLAM evaluator alongside the referee',
        ),
        DeclareLaunchArgument(
            'enable_perception',
            default_value='false',
            description='Set true to start the ROAR perception benchmark nodes',
        ),
        DeclareLaunchArgument(
            'enable_pursuit',
            default_value='false',
            description='Set true to start the Adaptive Pure Pursuit controller + monitor',
        ),
        OpaqueFunction(function=launch_setup),
    ])
