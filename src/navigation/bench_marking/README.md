# 📈 ROAR 2026 — Rover Benchmarking & Telemetry Evaluator Suite

[![Module](https://img.shields.io/badge/Module-navigation-E10600?logo=ros&logoColor=white)](file:///home/draaven/ROAR_2026/src/navigation)
[![Benchmarking](https://img.shields.io/badge/Metrics-ATE_|_RPE_|_Precision-00D9FF)](file:///home/draaven/ROAR_2026/reports)

The `bench_marking` directory hosts specialized evaluation nodes designed to scientifically measure the accuracy, drift, and perception efficacy of the rover navigation stack against simulated ground truth.

---

## 📦 Benchmark Nodes

```text
src/navigation/bench_marking/
├── roar_slam_benchmark/          # Trajectory accuracy & drift benchmark node
│   ├── roar_slam_benchmark/      # Python source (evaluator node, metrics calculator)
│   ├── package.xml
│   └── setup.py
│
├── roar_perception_testing/      # Perception obstacle clustering & ArUco detection accuracy
│   ├── roar_perception_testing/
│   ├── package.xml
│   └── setup.py
│
└── AdaptivePurePursuit/          # Baseline geometric path-tracking controller benchmark
    ├── src/
    ├── include/
    ├── package.xml
    └── CMakeLists.txt
```

---

## 🎯 Evaluation Metrics

### 1. SLAM & State Estimation (`roar_slam_benchmark`)
Compares the estimated state `/odom` (produced by the IESKF filter) and `/map` frame against Gazebo ground truth `/world/rover_world/pose/info`:
- **Absolute Trajectory Error (ATE)**: Measures global consistency and drift across long traversals:
  $$\text{ATE} = \sqrt{\frac{1}{N} \sum_{i=1}^N \|\mathbf{p}_{\text{est}, i} - \mathbf{p}_{\text{gt}, i}\|^2}$$
- **Relative Pose Error (RPE)**: Measures local velocity drift over fixed time windows ($\Delta t = 1\,\text{s}$).

### 2. Perception Accuracy (`roar_perception_testing`)
Evaluates LiDAR point-cloud clustering and camera detections against `obstacle_data.npy`:
- **Detection Precision & Recall**:
  $$\text{Precision} = \frac{TP}{TP + FP}, \quad \text{Recall} = \frac{TP}{TP + FN}$$
- **Centroid Offset Error**: Distance between estimated rock centroid and true rock center.

### 3. Path Following Performance (`AdaptivePurePursuit`)
- **Cross-Track Error (CTE)**: Orthogonal distance from planned path centerline.
- **Heading Error**: Angular deviation between rover orientation and path tangent vector.

