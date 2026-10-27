# 🧭 ROAR 2026 — Navigation Mission Control & Referee Suite

This module serves as the central mission orchestration, evaluation, and benchmarking hub for the ERC Navigation Task.

---

## 📁 Directory Layout

```text
src/navigation/
├── bench_marking/            # Performance evaluators
│   ├── AdaptivePurePursuit/  # Geometric path tracking controller benchmark
│   ├── roar_perception_testing/ # LiDAR clustering & ArUco detection accuracy benchmark
│   └── roar_slam_benchmark/  # Absolute/Relative Pose Error (APE/RPE) trajectory evaluator
├── missions/                 # Ground truth datasets & waypoints for Worlds 1..3
├── referee_engine/           # Automated ERC 2026 rules scoring engine (300-pt scoring)
└── scripts/                  # Mission Control GUI & preview utilities
    ├── asu_roar_gui.py       # Full PyQt5 telemetry & mission control dashboard
    └── preview_mission.py    # Waypoint & heightmap overlay preview renderer
```

---

## 🎮 Launching Mission Control GUI

```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/navigation/scripts/asu_roar_gui.py
```

The Mission Control GUI provides:
- **01 PREVIEW**: Visual inspection of terrain, obstacles, and waypoints.
- **02 EVALUATION**: Live telemetry, positional error $e_{\text{pos}}$, and live point calculation.
- **03 MISSION SETUP**: World, waypoint, and degradation parameter selection.
- **04 LAUNCH**: One-click orchestration of Gazebo, Nav2, and Referee nodes.
- **05 REPORTS**: Live scorecard rendering (`ERC_Report_Live.html`) and historical batch analytics.
