# 🚀 ASU ROAR 2026 — Autonomous Rover Simulation & Navigation Stack

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![Build](https://img.shields.io/badge/Build-colcon-success)](https://colcon.readthedocs.io/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)](https://www.python.org/)

Unified monorepo for the **ASU ROAR Team** competing in the **European Rover Challenge (ERC) 2026**.

This repository brings together the digital twin simulation, autonomous navigation stack, and automated mission evaluation into a single, fully buildable ROS 2 Humble workspace with zero external workspace dependencies.

---

## 🏛️ Repository Architecture

```text
ROAR_2026/
├── .gitignore                        # Git ignore rules for ROS 2 build artifacts & cache
├── README.md                         # Master documentation & quick-start guide
├── reports/                          # Auto-generated benchmark scorecards & telemetry logs
│   ├── referee/                      # ERC Referee evaluation summaries & live HTML reports
│   ├── slam/                         # IESKF trajectory accuracy & drift benchmarks
│   ├── perception/                   # GPU LiDAR clustering & ArUco detection metrics
│   └── pursuit/                      # Path-tracking error analysis
│
└── src/                              # The single ROS 2 Humble source directory
    ├── roar2026/                     # [1] Digital Twin, Rover Dynamics & Marsyard Simulation
    │   ├── coordinate_bridge/        # Metric survey-to-world coordinate transform bridge
    │   ├── dev_environment/         # Rover models (clean, noise, full) & generated test worlds
    │   ├── marsyards/                # 2026 Marsyard terrain model (erc_marsyard_2026) & ArUco tags 1..15
    │   ├── navMission_setup/         # Offline world & waypoint generation automation pipeline
    │   └── panel/                    # ERC maintenance panel simulation & 12 joint/switch controllers
    │
    ├── navigation/                   # [2] Mission Orchestration, GUI & Referee Evaluation
    │   ├── bench_marking/            # Evaluation nodes (AdaptivePurePursuit, roar_perception_testing, roar_slam_benchmark)
    │   ├── missions/                 # Waypoints and ground-truth assets for Worlds 1..3
    │   ├── referee_engine/           # Automated ERC 2026 scoring rules engine (300-pt traverse scoring)
    │   └── scripts/                  # Mission Control GUI (asu_roar_gui.py) & preview_mission.py
    │
    └── navstack/                     # [3] Autonomous Navigation & Motion Planning Stack
        ├── control/                  # MPPI local trajectory rollouts (nav2_mppi_controller + bundled local_deps)
        ├── navstack_nav2/            # Nav2 orchestration, costmap configurations, and behavior trees
        ├── path-planning/            # D* Lite replanning global planner (dstar_navigation)
        ├── perception/               # GPU accelerated LiDAR Euclidean clustering & 3D ArUco visual detection
        └── slam/                     # IESKF localization (roar_localization), 2.5D active terrain mapping & custom msgs
```

---

## 📦 Packages Index (29 Unique ROS 2 Packages)

| Package | Category | Build Type | Description |
| :--- | :--- | :--- | :--- |
| `active_mapping_msgs` | Navigation | `rosidl` | Custom ROS 2 IDL messages for 2.5D active elevation mapping |
| `roar_msgs` | Navigation | `rosidl` | Custom telemetry and state estimation message interfaces |
| `dstar_navigation` | Navigation | `ament_cmake` | D* Lite global path planner and offline waypoint pipeline |
| `nav2_mppi_controller` | Navigation | `ament_cmake` | Model Predictive Path Integral controller with bundled SIMD math (`local_deps`) |
| `adaptive_pure_pursuit`| Navigation | `ament_cmake` | Geometric path-tracking controller benchmark suite |
| `coordinate_bridge` | Simulation | `ament_python`| Coordinate frame bridge matching survey and Gazebo frames |
| `roar_rover_clean` | Simulation | `ament_cmake` | High-fidelity rover model with ideal kinematic physics |
| `roar_rover_noise` | Simulation | `ament_cmake` | Rover model with active sensor noise degradation |
| `roar_simulation_full` | Simulation | `ament_cmake` | Full rover model with 6-DOF robotic manipulator & degradation |
| `aruco_description` | Panel | `ament_cmake` | ArUco tag visual and collision model on maintenance panel |
| `electromagnet_description` | Panel | `ament_cmake` | Electromagnetic actuator simulation on panel |
| `erc_panel_sim` | Panel | `ament_cmake` | Main maintenance panel simulation environment |
| `led_indicator_description` | Panel | `ament_cmake` | LED indicator models with visual state feedback |
| `main_switch_description` | Panel | `ament_cmake` | Main electrical switch description and kinematics |
| `panel_body_description` | Panel | `ament_cmake` | Structural chassis and mounting plate description |
| `plate_description` | Panel | `ament_cmake` | Component mounting plates |
| `rotary_description` | Panel | `ament_cmake` | Continuous rotary knob controller models |
| `row_description` | Panel | `ament_cmake` | Arrayed terminal block component descriptions |
| `slab_description` | Panel | `ament_cmake` | Structural panel slab components |
| `socket_description` | Panel | `ament_cmake` | Power and signal socket models |
| `switch_description` | Panel | `ament_cmake` | Toggle switch models |
| `referee_engine` | Evaluation | `ament_python`| Automated ERC 2026 navigation scoring & report generator |
| `roar_perception_testing`| Evaluation | `ament_python`| Benchmark node for perception accuracy vs ground truth |
| `roar_slam_benchmark` | Evaluation | `ament_python`| Trajectory accuracy, drift, and APE/RPE metrics evaluator |
| `navstack_nav2` | Navigation | `ament_python`| Production Nav2 orchestration, lifecycle, and costmap configs |
| `perception_module` | Navigation | `ament_cmake` | GPU LiDAR obstacle detection and 3D ArUco pose estimation |
| `roar_localization` | Navigation | `ament_python`| Iterated Error-State Kalman Filter (IESKF) sensor fusion |
| `roar_slam_testing` | Navigation | `ament_python`| Real-time SLAM diagnostic evaluator node |
| `terrain_mapping` | Navigation | `ament_python`| Real-time 2.5D elevation and traversability costmap generator |

---

## ⚡ Quick Start

### 1. Build the Unified Monorepo
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ROAR_2026

# Sequential, memory-throttled build (crash-safe)
colcon build --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Source the newly built workspace
source install/setup.bash
```

### 2. Launch Mission Control GUI
The ASU ROAR Mission Control GUI orchestrates simulations, Nav2, benchmarks, and real-time reports:
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/navigation/scripts/asu_roar_gui.py
```

### 3. Launching via Command-Line (CLI)

#### Terminal 1 — Start Gazebo Simulation
```bash
cd ~/ROAR_2026/src/roar2026/dev_environment
bash launch_test.sh clean world1      # Options: clean | noise | full, world1 | world2 | world3
```

#### Terminal 2 — Start Autonomous Navigation Stack
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch navstack_nav2 roar_nav2_only.launch.py
```

#### Terminal 3 — Start Automated Referee Scoring Engine
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch referee_engine referee.launch.py mission:=mission00
```

---

## 🗺️ Mission & Waypoint Preview
Visualize waypoints overlaid on the Marsyard terrain and obstacle field:
```bash
python3 src/navigation/scripts/preview_mission.py --world 1 --wp 0
```
