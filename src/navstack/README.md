# 🛰️ ROAR 2026 — Autonomous Navigation & Motion Planning Stack

The autonomous navigation stack provides high-frequency local trajectory optimization, dynamic global replanning, GPU-accelerated perception, and multi-sensor fusion for the ROAR rover.

---

## 📁 Directory Layout

```text
src/navstack/
├── control/                  # nav2_mppi_controller (Model Predictive Path Integral controller)
│   └── nav2_mppi_controller/ # With bundled SIMD accelerated math in local_deps/
├── local_deps/               # Bundled XTL, XSimd, and XTensor header dependencies
├── navstack_nav2/            # Nav2 lifecycle manager, costmap layers & behavior trees
├── path-planning/            # Global planners
│   └── globalPlanner/        # dstar_navigation (D* Lite replanning algorithm)
├── perception/               # GPU perception
│   └── perception_module/    # CUDA/C++ Euclidean clustering & 3D ArUco pose estimation
├── scripts/                  # Autonomous run scripts (run_global.sh, run_local.sh)
└── slam/                     # State estimation & mapping
    ├── active_mapping_msgs/  # Custom elevation grid ROS 2 messages
    ├── roar_msgs/            # Telemetry interfaces
    ├── roar_localization/    # Iterated Error-State Kalman Filter (IESKF) sensor fusion
    ├── roar_slam_testing/    # Real-time SLAM diagnostic evaluator
    └── terrain_mapping/      # 2.5D active traversability heightmap generator
```

---

## 🚀 Running Nav2 Stack

```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch navstack_nav2 roar_nav2_only.launch.py
```
