# 🗺️ ROAR 2026 — D* Lite Global Path Planner

[![Package](https://img.shields.io/badge/Package-dstar__navigation-E10600?logo=ros&logoColor=white)](file://${ROAR_WS}/src/navstack/path-planning/globalPlanner)
[![Algorithm](https://img.shields.io/badge/Algorithm-D*_Lite_Replanning-00D9FF)](https://www.cs.cmu.edu/~maxim/files/dlite_aaai02.pdf)

The `dstar_navigation` package provides dynamic global path planning across the 2.5D Marsyard terrain. It implements the **D* Lite** algorithm (Koenig & Likhachev), enabling rapid replanning from scratch whenever the rover's on-board LiDAR detects new or moving obstacles.

---

## 🧭 Why D* Lite for Martian Navigation?

Unlike static planners (A* or Dijkstra), which must recompute the entire path from scratch when an unexpected obstacle is discovered, **D* Lite** operates backwards from the goal and repairs only the affected search graph nodes:

```mermaid
flowchart LR
    classDef step fill:#1A1A24,stroke:#00D9FF,stroke-width:1.5px,color:#FFFFFF;
    classDef alert fill:#3B1A1A,stroke:#E10600,stroke-width:2px,color:#FFFFFF;

    A["Compute Initial Global Path to Waypoint"] --> B["Rover Traverses Path"]
    B --> C{"LiDAR Detects Unseen Rock"}
    C -- "Path Blocked" --> D["Local Edge Cost Update c(u, v)"]
    D --> E["Fast Heuristic Inconsistent Vertex Repair"]
    E --> F["Immediate Feasible Path Replanned (< 10 ms)"]
    F --> B
    C -- "Path Clear" --> G["Arrive at Waypoint"]

    class A,B,D,E,F,G step;
    class C alert;
```

---

## 📁 Directory Structure

```text
src/navstack/path-planning/globalPlanner/
├── CMakeLists.txt                    # C++ build configuration
├── package.xml                       # ROS 2 Humble package definition
├── include/
│   ├── dstar_lite.h                  # Core C++ D* Lite graph search implementation
│   └── grid_map.h                    # 2D/2.5D grid map representation
├── src/
│   ├── dstar_node.cpp                # ROS 2 node interfacing with /global_plan
│   └── dstar_lite.cpp                # Algorithmic priority queue operations
├── config/
│   └── config.yaml                   # Grid resolution, inflation radius & heuristic weights
├── scripts/
│   ├── erc_waypoint_follower.py      # Interactive waypoint follower dispatching goals to Nav2
│   └── batch_waypoint_follower.py    # Headless batch test runner for automated evaluations
└── offline_tools/
    └── run_pipeline.sh               # Standalone heightmap-to-plan generation pipeline
```

---

## ⚡ ROS 2 Node Interface

- **Subscribes**:
  - `/map` (`nav_msgs/OccupancyGrid`): Active static and dynamic costmap layers.
  - `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`): Current rover localization.
  - `/goal_pose` (`geometry_msgs/PoseStamped`): Waypoint target pose.
- **Publishes**:
  - `/global_plan` (`nav_msgs/Path`): Collision-free trajectory delivered to the MPPI local controller.

