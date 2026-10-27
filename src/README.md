# 🛰️ ASU ROAR 2026 — End-to-End Autonomous Rover Pipeline Architecture

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![Target](https://img.shields.io/badge/Competition-ERC_2026-E10600?logo=target&logoColor=white)](https://roverchallenge.eu/)
[![Status](https://img.shields.io/badge/Verification-100%25_Passing-00D9FF?logo=checkmarx&logoColor=white)](../)

Welcome to the central technical documentation for the **ASU ROAR 2026 Autonomous Rover System**. This guide provides an end-to-end visual and architectural overview of our software stack, simulation environment, and autonomous navigation pipeline developed for the **European Rover Challenge (ERC) 2026**.

## Quick Start — Run From Any Location

The project does not require a specific username, parent directory, or workspace
name. After cloning or downloading it, enter its `src` directory and run the
portable launcher:

```bash
cd ROAR-26-Simulation/src
./start_gui.sh
```

For example, the project can be stored in any location:

```bash
cd ~/Downloads/ROAR-26-Simulation/src
./start_gui.sh
```

On the first run, `start_gui.sh` automatically detects the workspace root,
sets `ROAR_WS`, sources ROS 2 Humble, builds the workspace, sources the built
overlay, and launches the Mission Control GUI. Later runs reuse the existing
build and start the GUI directly.

### Requirements

- Ubuntu with ROS 2 Humble installed under `/opt/ros/humble`
- The project dependencies required by its ROS packages
- An available graphical display for the Mission Control GUI

## Using the Mission Control GUI

For a safe first run, use World 1, waypoint 0, one iteration, and the
**Noise Free** rover.

1. Open **01 — PREVIEW**.
   - Set `WORLD` to `1`.
   - Set `WP INDEX` to `0`.
   - Click **RENDER MAP** and confirm the terrain and route appear.

2. Open **02 — EVALUATION**.
   - Keep **SLAM EVALUATION ENABLED** selected.
   - Leave Perception and Pure Pursuit evaluation disabled for the first run.

3. Open **03 — MISSION SETUP**.
   - Set `FROM WORLD` and `TO WORLD` to `1`.
   - Set `FROM WP` and `TO WP` to `0`.
   - Set `ITERATIONS` to `1`.
   - Click **ADD TO BATCH QUEUE**.

4. Open **04 — LAUNCH**.
   - Select **Noise Free** under `ROVER CONFIG`.
   - Optionally select **Open RViz2**.
   - Click **RUN BATCH PIPELINE**.

5. Watch **RUN STATUS MONITOR** while the GUI starts the pipeline:

   ```text
   Gazebo → Nav2/SLAM → Referee → Waypoint follower → Reports
   ```

6. After completion, open **05 — REPORTS** to view the master evaluation and
   subsystem reports.

### Stopping the System

- Use **CLEAR & RESET BATCH (KILL NODES)** to cancel a batch and clear its queue.
- Use **TERMINATE ALL** to stop manually launched simulation and navigation processes.
- Use **SKIP CURRENT TEST** to skip only the active batch item.

### Manual Launch Mode

In **04 — LAUNCH**, use the manual buttons in this order:

1. Click **1. LAUNCH SIM**.
2. Wait for **SIMULATION READY**.
3. Click **2. LAUNCH NAV2** and wait for Nav2 to become active.
4. Click **3. SEND WPTS**.
5. Click **TERMINATE ALL** when finished.

---

## 📑 Table of Contents
1. [Executive Summary & System Workflow](#-executive-summary--system-workflow)
2. [End-to-End Dataflow & Pipeline Architecture](#-end-to-end-dataflow--pipeline-architecture)
3. [ROS 2 Computational Graph & Topic Wiring](#-ros-2-computational-graph--topic-wiring)
4. [Coordinate Reference Frames & Transformation Bridge](#-coordinate-reference-frames--transformation-bridge)
5. [Autonomous Navigation Stack Breakdown](#-autonomous-navigation-stack-breakdown)
6. [Automated Referee & Evaluation Pipeline](#-automated-referee--evaluation-pipeline)
7. [Module Breakdown & Package Inventory](#-module-breakdown--package-inventory)

---

## 🌐 Executive Summary & System Workflow

The ROAR 2026 system is composed of three interconnected modules hosted inside a single, unified ROS 2 Humble workspace:

```text
${ROAR_WS}/src/
├── roar2026/       --> Physics Engine, Digital Twin, Rover Models & Environment Generation
├── navigation/     --> Mission Control Dashboard, Benchmark Suites & ERC Referee Engine
└── navstack/       --> Autonomous Navigation (Global D* Lite, Local MPPI, SLAM, GPU Perception)
```

### High-Level System Workflow
```mermaid
flowchart TD
    classDef sim fill:#1A1A24,stroke:#E10600,stroke-width:2px,color:#FFFFFF;
    classDef nav fill:#1A241A,stroke:#00D9FF,stroke-width:2px,color:#FFFFFF;
    classDef eval fill:#241A1A,stroke:#FFB300,stroke-width:2px,color:#FFFFFF;
    classDef ui fill:#241A24,stroke:#BB86FC,stroke-width:2px,color:#FFFFFF;

    subgraph SIMULATION ["1. SIMULATION & DIGITAL TWIN (src/roar2026)"]
        A1["2026 Marsyard Terrain (erc_marsyard_2026)"] --> A2["Generated Test Worlds (world1..3)"]
        A3["Rover Kinematics & Physics (Diff Drive + 6-DOF Arm)"] --> A4["Sensor Degradation Engine (Dust, Glare, Noise)"]
        A2 --> A5["Gazebo Fortress Physics Engine"]
        A4 --> A5
    end

    subgraph NAVSTACK ["2. AUTONOMOUS NAVIGATION (src/navstack)"]
        B1["Sensors: 3D LiDAR, ZED 2i Stereo, BNO055 IMU, Encoders"]
        B2["GPU Perception (CUDA Clustering + 3D ArUco)"]
        B3["IESKF Sensor Fusion (roar_localization)"]
        B4["2.5D Active Terrain Mapping (terrain_mapping)"]
        B5["D* Lite Global Replanner (dstar_navigation)"]
        B6["MPPI Trajectory Rollouts (nav2_mppi_controller)"]
        
        B1 --> B2 & B3
        B3 --> B4
        B4 --> B5
        B5 --> B6
        B2 --> B6
    end

    subgraph EVALUATION ["3. REFEREE & BENCHMARKING (src/navigation)"]
        C1["ERC 2026 Referee Scoring Engine (referee_engine)"]
        C2["SLAM Drift & Trajectory Benchmarks (roar_slam_benchmark)"]
        C3["Perception Accuracy Benchmarks (roar_perception_testing)"]
        C4["Pursuit Tracking Accuracy (AdaptivePurePursuit)"]
    end

    subgraph DASHBOARD ["4. MISSION CONTROL (src/navigation/scripts)"]
        D1["ASU ROAR Mission Control GUI (asu_roar_gui.py)"]
        D2["Real-Time Live HTML Scorecards (ERC_Report_Live.html)"]
        D3["Master Batch Evaluation Reports (master_report_batch.html)"]
    end

    A5 -- "ros_gz_bridge (Clock, TF, Sensors)" --> B1
    B6 -- "/cmd_vel commands" --> A5
    A5 -- "Ground Truth TF" --> C1 & C2
    B3 -- "Estimated Odometry" --> C1 & C2
    C1 --> D2
    C2 & C3 & C4 --> D3
    D1 -- "Orchestration & Run Control" --> A5 & B5 & C1

    class A1,A2,A3,A4,A5 sim;
    class B1,B2,B3,B4,B5,B6 nav;
    class C1,C2,C3,C4 eval;
    class D1,D2,D3 ui;
```

---

## 🔄 End-to-End Dataflow & Pipeline Architecture

The entire progression from raw photogrammetry survey meshes down to real-time waypoint execution and referee reporting operates through the following deterministic pipeline:

```mermaid
sequenceDiagram
    autonumber
    actor Team as Team Operator
    participant GUI as Mission Control GUI
    participant Sim as Gazebo Fortress (roar2026)
    participant Sensors as Sensor Degradation
    participant SLAM as IESKF & Mapping (navstack)
    participant Planner as D* Lite & MPPI (navstack)
    participant Ref as Referee Engine (navigation)

    Team->>GUI: Select Rover Config (noise), World (1), Waypoints
    GUI->>Sim: Launch Simulation (launch_test.sh)
    Sim->>Sensors: Generate Physical Terrain Contacts & Raw Measurements
    Sensors->>SLAM: Degraded PointCloud2, IMU, Encoder Pulses
    SLAM->>SLAM: Fuse IESKF State -> Publish map->odom TF & /odom
    SLAM->>GUI: Telemetry Heartbeat (Simulation Ready: /odom active)
    GUI->>Planner: Launch Nav2 Stack (roar_nav2_only.launch.py)
    Planner->>Planner: Wait for Nav2 Lifecycle Manager (/navigate_to_pose active)
    Planner->>GUI: Nav2 Ready Signal
    GUI->>Ref: Launch Referee Engine with Mission Specs
    GUI->>Planner: Dispatch Waypoint Goals in Sequence
    
    loop Traversal Loop
        Planner->>Planner: D* Lite Global Plan -> MPPI Optimized Rollouts
        Planner->>Sim: Stream /cmd_vel (Linear & Angular Velocity)
        Sim->>Ref: Publish Ground Truth TF (/world/rover_world/pose/info)
        SLAM->>Ref: Publish Estimated Pose (/odom)
        Ref->>Ref: Calculate e_pos, Edge Clearance d, Speed Limits
        Ref->>GUI: Stream Live Scorecard (ERC_Report_Live.html)
    end

    Ref->>GUI: Waypoints Complete -> Final Score & Penalty Calculation
    GUI->>Team: Display Comprehensive Master Batch Analytics
```

---

## 🔌 ROS 2 Computational Graph & Topic Wiring

The table below documents the core topic contracts bridging simulation, navigation, and referee scoring:

```mermaid
graph LR
    subgraph GAZEBO ["Gazebo Fortress Simulation"]
        G_CLOCK["/clock (rosgraph_msgs/Clock)"]
        G_GT["/world/rover_world/pose/info (tf2_msgs/TFMessage)"]
        G_IMU["/bno055/data (sensor_msgs/Imu)"]
        G_LIDAR["/lidar/points (sensor_msgs/PointCloud2)"]
        G_CAM["/zed2i/left/image_rect_color (sensor_msgs/Image)"]
        G_DIFF["/diff_drive_controller/odom (nav_msgs/Odometry)"]
        G_CMD["/cmd_vel (geometry_msgs/Twist)"]
    end

    subgraph BRIDGES ["Bridge & Noise Layer"]
        B_GT["ground_truth_bridge"]
        B_ENC["encoder_sim_node"]
        B_DEG["zed_degradation_node"]
    end

    subgraph NAV ["Autonomous Navigation Stack"]
        N_LOC["roar_localization (IESKF)"]
        N_MAP["terrain_mapping (2.5D)"]
        N_PERC["perception_module (CUDA)"]
        N_GLOBAL["dstar_navigation (D* Lite)"]
        N_LOCAL["nav2_mppi_controller"]
    end

    subgraph REF ["Referee & Evaluation"]
        R_CORE["referee_core_node"]
        R_SLAM["roar_slam_benchmark"]
    end

    G_GT --> B_GT -->|Fake ZED Pose| N_LOC
    G_GT --> R_CORE & R_SLAM
    G_IMU --> N_LOC
    G_DIFF --> B_ENC -->|Encoder Pulses| N_LOC
    G_LIDAR --> N_PERC & N_MAP
    G_CAM --> B_DEG --> N_PERC

    N_LOC -->|/odom + map->odom TF| N_MAP & N_GLOBAL & N_LOCAL & R_CORE & R_SLAM
    N_PERC -->|Obstacle Clusters & ArUco 3D| N_LOCAL & N_GLOBAL
    N_MAP -->|Local Costmap Layers| N_LOCAL
    N_GLOBAL -->|/global_plan| N_LOCAL
    N_LOCAL -->|/cmd_vel| G_CMD
```

---

## 📐 Coordinate Reference Frames & Transformation Bridge

One of the most critical aspects of the European Rover Challenge is handling discrepancies between the **Official Survey Reference Frame** and the **Gazebo Simulation Frame**:

### Coordinate System Transformation
```text
Official ERC Survey Frame                     Gazebo Fortress Frame
           +Y                                          +X
           ▲                                           ▲
           │                                           │
           │                                           │
  -X ◄─────┼─────► +X                 -Y ◄─────────────┼─────────────► +Y
           │                                           │
           │                                           │
           ▼ -Y                                        ▼ -Z
```

$$\begin{aligned}
X_{\text{gazebo}} &= X_{\text{reference}} \\
Y_{\text{gazebo}} &= -Y_{\text{reference}} \\
Z_{\text{gazebo}} &= Z_{\text{reference}}
\end{aligned}$$

- **`src/roar2026/coordinate_bridge`** provides a centralized, tested Python API (`survey_to_world()` / `world_to_survey()`) to guarantee that all metric coordinates (waypoints, obstacles, ArUco positions) are seamlessly mapped without manual negation bugs.

---

## 🚗 Autonomous Navigation Stack Breakdown

```mermaid
flowchart TD
    classDef input fill:#1E293B,stroke:#38BDF8,stroke-width:1.5px,color:#F8FAFC;
    classDef process fill:#0F172A,stroke:#818CF8,stroke-width:1.5px,color:#F8FAFC;
    classDef output fill:#1E1B4B,stroke:#C084FC,stroke-width:2px,color:#F8FAFC;

    subgraph SENSORS ["Perception & Localization Inputs"]
        S1["3D LiDAR Point Cloud (16-beam Velodyne)"]
        S2["ZED 2i Stereo Camera (Degraded RGBA + Depth)"]
        S3["BNO055 9-DOF IMU (Angular Rates + Linear Accel)"]
        S4["Simulated Wheel Encoders (~201,554 pulses/rev)"]
    end

    subgraph FUSION ["Localization & Mapping Pipeline"]
        F1["IESKF Sensor Fusion (roar_localization)"]
        F2["2.5D Elevation & Traversability Mapping (terrain_mapping)"]
        F3["GPU Euclidean Clustering (perception_module)"]
        F4["3D ArUco Marker Detector (perception_module)"]
    end

    subgraph PLANNING ["Nav2 Motion Planning"]
        P1["Global Costmap (Static Terrain + Danger Zones)"]
        P2["D* Lite Replanner (dstar_navigation)"]
        P3["Local Costmap (Dynamic LiDAR + Rolling Grids)"]
        P4["MPPI Controller (nav2_mppi_controller with SIMD Critics)"]
    end

    subgraph ACTUATION ["Robot Actuation"]
        CMD["/cmd_vel (Twist: linear.x, angular.z)"]
        DIFF["DiffDriveController (Hardware Torque Simulation)"]
    end

    S3 & S4 --> F1
    S1 --> F2 & F3
    S2 --> F4
    F1 -->|map->odom TF| P1 & P3
    F2 --> P1 & P3
    F3 --> P3
    P1 --> P2
    P2 -->|Reference Global Path| P4
    P3 --> P4
    P4 --> CMD --> DIFF

    class S1,S2,S3,S4 input;
    class F1,F2,F3,F4,P1,P2,P3,P4 process;
    class CMD,DIFF output;
```

---

## ⚖️ Automated Referee & Evaluation Pipeline

The **`referee_engine`** executes official ERC rules to score rover navigation in real time:

$$\text{Final Score} = \sum_{i=1}^{N_{\text{legs}}} \text{Points}_i - \text{Penalties}$$

### Scoring Rules Breakdown:
1. **Positional Error Deduction**:
   $$e_{\text{pos}} = \|\mathbf{p}_{\text{rover}} - \mathbf{p}_{\text{wp}}\|_2$$
   - $e_{\text{pos}} \le 0.5\,\text{m}$: Full 100% leg score.
   - $0.5\,\text{m} < e_{\text{pos}} \le 2.0\,\text{m}$: Linear score degradation down to 20%.
   - $e_{\text{pos}} > 2.0\,\text{m}$: 0 points for leg.
2. **Speed & Time Budget**:
   - Computes $T_{\text{min}}$ (from max velocity $v_{\text{max}} = 0.8\,\text{m/s}$) and $T_{\text{max}}$ (from min velocity $v_{\text{min}} = 0.15\,\text{m/s}$).
   - Time exceeding $T_{\text{max}}$ applies continuous point decay; hard stop at 20 minutes (ERC regulation).
3. **Collision & Safety Penalties**:
   - Severe penalty ($-50$ pts) for rock collisions or leaving survey boundaries.

---

## 📁 Module Breakdown & Package Inventory

The monorepo contains **29 ROS 2 packages** structured into three clean source directories:

```text
${ROAR_WS}/src/
│
├── roar2026/                         # [Module 1: Simulation & Digital Twin]
│   ├── coordinate_bridge/            # Metric frame transform bridge
│   ├── dev_environment/             # Rover models (clean, noise, full) & test worlds 1..3
│   ├── marsyards/                    # erc_marsyard_2026 terrain model & ArUco 1..15 models
│   ├── navMission_setup/             # World & waypoint automated generation pipelines
│   └── panel/                        # ERC maintenance panel & 12 component controller packages
│
├── navigation/                       # [Module 2: Mission Control & Evaluation]
│   ├── bench_marking/                # Evaluators: AdaptivePurePursuit, roar_perception_testing, roar_slam_benchmark
│   ├── missions/                     # Mission assets & waypoint arrays for Worlds 1..3
│   ├── referee_engine/               # ERC automated scoring rules engine
│   └── scripts/                      # Mission Control GUI (asu_roar_gui.py) & preview_mission.py
│
└── navstack/                         # [Module 3: Autonomous Navigation Stack]
    ├── control/                      # nav2_mppi_controller (with bundled SIMD local_deps)
    ├── navstack_nav2/                # Nav2 orchestration, costmaps, and behavior tree configs
    ├── path-planning/                # dstar_navigation (D* Lite global replanner)
    ├── perception/                   # perception_module (GPU LiDAR clustering & 3D ArUco detection)
    └── slam/                         # roar_localization (IESKF), terrain_mapping, roar_slam_testing
```

---

## 📖 Sub-Module Technical Documentation

For in-depth documentation on individual modules and sub-systems, refer to their dedicated README guides:

| Module / Subsystem | Path | Description |
| :--- | :--- | :--- |
| **Simulation Core** | [`src/roar2026/README.md`](file://${ROAR_WS}/src/roar2026/README.md) | Physics engine, sensor degradation models, and Gazebo fortress configs |
| **Dev Environment** | [`src/roar2026/dev_environment/README.md`](file://${ROAR_WS}/src/roar2026/dev_environment/README.md) | CLI launch runner `launch_test.sh` and multi-world testing guide |
| **Marsyard & Models** | [`src/roar2026/marsyards/README.md`](file://${ROAR_WS}/src/roar2026/marsyards/README.md) | 2026 terrain mesh specifications, collision optimization, and ArUco tags |
| **World & Waypoint Pipeline** | [`src/roar2026/navMission_setup/README.md`](file://${ROAR_WS}/src/roar2026/navMission_setup/README.md) | Heightmap, costmap, and waypoint synthesis automation |
| **Maintenance Panel** | [`src/roar2026/panel/README.md`](file://${ROAR_WS}/src/roar2026/panel/README.md) | Maintenance panel URDFs, joints, switch controllers, and Gazebo interfaces |
| **Coordinate Bridge** | [`src/roar2026/coordinate_bridge/README.md`](file://${ROAR_WS}/src/roar2026/coordinate_bridge/README.md) | Reference frame vs Gazebo coordinate transformation API |
| **Navigation & GUI** | [`src/navigation/README.md`](file://${ROAR_WS}/src/navigation/README.md) | Mission Control GUI dashboard, scoring rules, and batch evaluation |
| **Referee Engine** | [`src/navigation/referee_engine/README.md`](file://${ROAR_WS}/src/navigation/referee_engine/README.md) | Official ERC rules engine, time budgets, and live scorecards |
| **Benchmarking Suite** | [`src/navigation/bench_marking/README.md`](file://${ROAR_WS}/src/navigation/bench_marking/README.md) | SLAM trajectory error (APE/RPE), perception accuracy, and pursuit benchmarks |
| **Mission Datasets** | [`src/navigation/missions/README.md`](file://${ROAR_WS}/src/navigation/missions/README.md) | Waypoints formats, obstacle datasets, and ground truth assets |
| **Autonomous NavStack** | [`src/navstack/README.md`](file://${ROAR_WS}/src/navstack/README.md) | Nav2 stack bringup, behavior trees, and navigation lifecycle |
| **MPPI Controller** | [`src/navstack/control/README.md`](file://${ROAR_WS}/src/navstack/control/README.md) | Local trajectory rollouts, SIMD critics, and dynamic parameter tuning |
| **D* Lite Global Planner** | [`src/navstack/path-planning/README.md`](file://${ROAR_WS}/src/navstack/path-planning/README.md) | Incremental graph search on 2.5D costmaps and obstacle avoidance |
| **GPU Perception** | [`src/navstack/perception/README.md`](file://${ROAR_WS}/src/navstack/perception/README.md) | CUDA Euclidean point-cloud clustering and 3D ArUco visual localization |
| **IESKF & SLAM** | [`src/navstack/slam/README.md`](file://${ROAR_WS}/src/navstack/slam/README.md) | Multi-sensor Kalman filtering, encoder odometry, and elevation mapping |
