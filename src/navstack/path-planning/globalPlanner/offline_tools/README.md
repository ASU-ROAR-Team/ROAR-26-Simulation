# 🗺️ Offline Tools Suite (`dstar_navigation`)

A unified, high-performance offline path planning, tour optimization, route editing, and attraction costmap generation suite designed for autonomous rover navigation in the ERC Marsyard environment.

---

## 📂 Directory Layout

```
offline_tools/
├── config.yaml                          # Central configuration (paths, resolution, attraction parameters)
├── run_pipeline.sh                      # Pipeline automation script
├── README.md                            # Suite documentation & usage manual
│
├── scripts/                             # Modular Python Scripts
│   ├── planning/
│   │   ├── offline_sequence_planner.py  # ROS 2 sequential D* global path & attraction layer generator
│   │   ├── dstar_tour_optimizer.py      # Pure C++ D* Lite TSP 24-tour & attraction layer optimizer
│   │   └── optimize_waypoint_tour.py    # Fast Dijkstra TSP tour optimizer
│   ├── analysis/
│   │   ├── path_editor.py               # Interactive Pygame GUI (WAYPOINT, DRAW mode, & auto attraction layer)
│   │   ├── compare_paths.py             # Matplotlib before/after path visualizer
│   │   ├── path_cost_evaluator.py       # Traversal cost & lethal obstacle reporter
│   │   └── filter_path.py               # RDP path simplification utility
│   └── mapping/
│       ├── generate_path_attraction_costmap.py # Standalone negative-cost corridor layer generator
│       ├── generate_heightmap.py        # STL 3D mesh -> heightmap_world.npz rasterizer
│       ├── convert_heightmap_to_costmap.py # Heightmap -> costmap.csv slope/step converter
│       ├── map_calibration.py           # Pixel to world coordinate calibrator
│       └── visualize_heightmap.py       # Heightmap visualizer utility
│
├── data/                                # Dynamic Output & Runtime Data Files
│   ├── heightmap_world.npz              # Primary Marsyard 3D elevation grid (419x274)
│   ├── costmap.csv                      # Active D* terrain costmap (0..100)
│   ├── combined_offline_path.csv        # Generated 3D trajectory (x, y, z)
│   ├── combined_offline_path_attraction_costmap.csv # Pure negative-cost attraction corridor layer
│   ├── edited_offline_path.csv          # User-edited path from path_editor.py
│   ├── edited_offline_path_attraction_costmap.csv   # Attraction layer for user-edited path
│   ├── optimal_waypoints.csv            # Optimal D* tour waypoint sequence
│   └── optimal_path_attraction_costmap.csv # Attraction layer for optimal D* tour
│
└── reference/                           # Static Reference & Ground-Truth Files
    ├── waypoints.csv                    # Ground-truth mission waypoints
    ├── baseline_costmap.csv              # Baseline ERC terrain costmap
    ├── real_path.csv                    # Real rover path reference
    └── Calibration_map.png              # Reference calibration image
```

---

## 🧲 Path Attraction Costmap Layer Feature

Instead of enforcing rigid waypoint constraints on the rover, the suite exports an **independent negative-cost attraction layer** along offline trajectories:
* **Centerline Bonus**: Configurable negative cost bias along path centerline (default: `-30.0` in `config.yaml`).
* **Gaussian Decay**: Smooth attraction corridor decaying over a configurable radius (default: `corridor_radius: 1.5m`, `sigma: 0.5m`).
* **Zero Outside**: `0.00` cost outside the path corridor.
* **Online Behavior**: Loaded on top of D\* Lite's costmap on the rover, D\* naturally clings to the offline track when clear, but dynamically avoids dynamic obstacles and returns to the path corridor afterwards.

---

## ⚙️ Configuration (`config.yaml`)

```yaml
# Path attraction costmap layer configuration
attraction_costmap:
  max_bonus_cost:   30.0   # Maximum negative cost bonus along path centerline (-30)
  corridor_radius:   1.5   # Maximum radius of attraction corridor in meters
  sigma:             0.5   # Gaussian decay sigma in meters
```

---

## 🚀 Usage Guide

### 1. Standalone Attraction Costmap Layer Generator
```bash
python3 path-planning/globalPlanner/offline_tools/scripts/mapping/generate_path_attraction_costmap.py \
  -i path-planning/globalPlanner/offline_tools/data/combined_offline_path.csv \
  --max-bonus 30.0 \
  --corridor-radius 1.5 \
  --sigma 0.5
```

---

### 2. Multi-Goal Tour Optimizer (Generates Optimal Tour + Attraction Layer)
```bash
# Terminal 1: Launch C++ D* Lite Action Server
ros2 run dstar_navigation dstar_node

# Terminal 2: Run Tour Optimizer
python3 path-planning/globalPlanner/offline_tools/scripts/planning/dstar_tour_optimizer.py
```

---

### 3. ROS 2 Offline Sequence Planner (Generates Sequence + Attraction Layer)
```bash
ros2 launch dstar_navigation offline_planner.launch.py
```

---

### 4. Interactive Path Editor (Saves Edited Path + Attraction Layer)
```bash
python3 path-planning/globalPlanner/offline_tools/scripts/analysis/path_editor.py
```
* Press **`S`** to save both `data/edited_offline_path.csv` AND `data/edited_offline_path_attraction_costmap.csv`.
