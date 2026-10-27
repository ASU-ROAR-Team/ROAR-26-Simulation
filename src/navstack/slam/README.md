# 🧭 ROAR 2026 — SLAM, IESKF Localization & Active Mapping

[![Module](https://img.shields.io/badge/Module-navstack-E10600?logo=ros&logoColor=white)](file://${ROAR_WS}/src/navstack)
[![Filter](https://img.shields.io/badge/Filter-IESKF_Fusion-00D9FF)](file://${ROAR_WS}/src/navstack/slam/roar_localization)

This subsystem provides robust state estimation and real-time 2.5D elevation mapping for the ROAR rover over rough Martian terrain where GPS is unavailable.

---

## 🧮 Iterated Error-State Kalman Filter (IESKF) Architecture

The `roar_localization` package executes an **IESKF** algorithm that decouples nominal kinematic integration from error-state estimation:

$$\mathbf{x} = \begin{bmatrix} \mathbf{p}^T & \mathbf{v}^T & \mathbf{q}^T & \mathbf{b}_a^T & \mathbf{b}_g^T \end{bmatrix}^T \in \mathbb{R}^{16}$$

```mermaid
flowchart LR
    classDef input fill:#1E293B,stroke:#38BDF8,stroke-width:1.5px,color:#F8FAFC;
    classDef filter fill:#0F172A,stroke:#818CF8,stroke-width:1.5px,color:#F8FAFC;
    classDef output fill:#1E1B4B,stroke:#10B981,stroke-width:2px,color:#F8FAFC;

    subgraph MEASUREMENTS ["Multi-Rate Sensor Ingestion"]
        M1["BNO055 IMU: Angular Velocity & Acceleration (100 Hz)"]
        M2["Wheel Encoders: ~201,554 pulses/rev (50 Hz)"]
        M3["ZED 2i Visual Odometry / ArUco Poses (15 Hz)"]
    end

    subgraph IESKF ["IESKF Core Engine (roar_localization)"]
        K1["High-Rate Nominal Kinematic Propagation"]
        K2["Iterative Gauss-Newton Measurement Update"]
        K3["Manifold SO(3) Quaternion Error Injection"]
    end

    subgraph ESTIMATES ["Outputs & TF Broadcasts"]
        O1["/odom (nav_msgs/Odometry @ 50 Hz)"]
        O2["TF: map -> odom -> base_link"]
        O3["Active 2.5D Mapping Grid (terrain_mapping)"]
    end

    M1 --> K1
    M2 & M3 --> K2
    K1 --> K2 --> K3 --> O1 & O2 & O3

    class M1,M2,M3 input;
    class K1,K2,K3 filter;
    class O1,O2,O3 output;
```

---

## 📦 Sub-Packages Breakdown

```text
src/navstack/slam/
├── active_mapping_msgs/      # Custom ROS 2 IDL messages for elevation matrices
├── roar_msgs/                # Custom telemetry, covariance & benchmark message types
├── roar_localization/        # Production IESKF multi-sensor fusion node (ieskf_node.py)
├── roar_slam_testing/        # Real-time SLAM diagnostic evaluator & trajectory visualizer
└── terrain_mapping/          # Online 2.5D elevation and slope roughness map generator
```

---

## 🗺️ 2.5D Active Terrain Mapping (`terrain_mapping`)

As the rover traverses the terrain, `terrain_mapping` continuously integrates incoming LiDAR point clouds into a rolling 2.5D elevation grid:
- **Cell Resolution**: $0.10\,\text{m} \times 0.10\,\text{m}$.
- **Grid Features**:
  - Mean elevation $z_{\text{mean}}$.
  - Local surface variance (roughness $\sigma_z^2$).
  - Surface normal angle $\theta_{\text{slope}} = \arccos(\mathbf{n} \cdot \mathbf{k})$.
- **Costmap Integration**: Automatically published to Nav2 costmap layers to steer the local planner away from steep crater slopes.
