# Global Planner

Offline global path-planning pipeline for the ERC navigation system.

The package combines terrain/heightmap processing, waypoint-based mission planning, and D* global path planning.

---

## Architecture

```text
                    ┌──────────────────────┐
                    │   Waypoint Generator │
                    │                      │
                    │ costmap_*.npz        │
                    │ obstacle_data_*.npy  │
                    └──────────┬───────────┘
                               │
                         waypoint sets
                               │
                               ▼
                    ┌──────────────────────┐
                    │ Offline Sequence     │
                    │ Planner              │
                    │                      │
                    │ heightmap + waypoints│
                    └──────────┬───────────┘
                               │
                         D* goal requests
                               │
                               ▼
                    ┌──────────────────────┐
                    │ D* Global Planner    │
                    │                      │
                    │ dstar_node           │
                    │ dstar_planner        │
                    └──────────┬───────────┘
                               │
                           /global_plan
                               │
                               ▼
                    ┌──────────────────────┐
                    │ RViz / Path Output   │
                    └──────────────────────┘
```

### Main flow

1. A heightmap is loaded by `offline_sequence_planner.py`.
2. The heightmap is converted into a planner costmap.
3. Waypoints are loaded from a CSV file.
4. Each consecutive waypoint pair becomes a D* planning leg.
5. `offline_sequence_planner.py` sends the current leg to `dstar_node`.
6. D* calculates the global path.
7. The resulting path is received through `/global_plan`.
8. The planner continues until all waypoint legs are completed.

---

# Directory Structure

```text
globalPlanner/
├── CMakeLists.txt
├── dstar_planner.xml
│
├── src/
│   ├── dstar_planner.cpp
│   └── dstar_node.cpp
│
├── launch/
│   └── offline_planner.launch.py
│
├── offline_tools/
│   ├── scripts/
│   │   ├── planning/
│   │   │   ├── offline_sequence_planner.py
│   │   │   ├── dstar_tour_optimizer.py
│   │   │   └── optimize_waypoint_tour.py
│   │   │
│   │   ├── mapping/
│   │   │   ├── generate_heightmap.py
│   │   │   ├── convert_heightmap_to_costmap.py
│   │   │   ├── generate_path_attraction_costmap.py
│   │   │   └── ...
│   │   │
│   │   └── analysis/
│   │       ├── path_editor.py
│   │       ├── path_cost_evaluator.py
│   │       └── compare_paths.py
│   │
│   └── data/
│       ├── heightmap_visualization.png
│       ├── costmap.csv
│       ├── combined_offline_path.csv
│       └── ...
│
├── inputs/
│   ├── heightmap00.npz
│   └── wp00_191.csv
│
└── test/
```

---

# Important Files

## `src/dstar_node.cpp`

ROS 2 node that exposes the D* planner to the rest of the system.

Responsibilities:

* receives planning goals
* receives the map/costmap
* converts planning requests into D* operations
* publishes the resulting global path
* reports planning failures

Node:

```text
dstar_global_planner
```

---

## `src/dstar_planner.cpp`

Core D* planning implementation.

Responsible for:

* maintaining the search grid
* calculating path costs
* handling map updates
* finding a path between start and goal

This is the actual planning algorithm; `dstar_node.cpp` is the ROS interface around it.

---

## `offline_tools/scripts/planning/offline_sequence_planner.py`

Main offline planning orchestrator.

Responsibilities:

* loads the waypoint CSV
* loads the heightmap
* creates/publishes map layers
* sends sequential goals to D*
* waits for the generated `/global_plan`
* processes each waypoint leg
* generates the complete offline path

Important parameters:

```text
waypoints_file
heightmap_file
```

Example:

```bash
ros2 launch dstar_navigation offline_planner.launch.py \
    waypoints_file:=wp00.csv \
    heightmap_file:=heightmap00.npz
```

---

## `launch/offline_planner.launch.py`

Starts the complete offline planning environment.

Currently launches:

```text
dstar_node
offline_sequence_planner.py
static_transform_publisher
rviz2
```

It provides:

```text
waypoints_file
heightmap_file
```

as launch arguments.

---

# Mapping Tools

### `generate_heightmap.py`

Generates the terrain heightmap used by the planner.

### `convert_heightmap_to_costmap.py`

Converts height information into a planning cost representation.

### `generate_path_attraction_costmap.py`

Generates an attraction layer that can influence path selection.

### `visualize_heightmap.py`

Visualizes the generated terrain heightmap.

### `visualize_attraction_costmap.py`

Visualizes the attraction costmap.

---

# Analysis Tools

### `path_cost_evaluator.py`

Evaluates the cost of generated paths.

### `compare_paths.py`

Compares different generated paths.

### `path_editor.py`

Allows offline modification/editing of generated paths.

---

# Inputs

Typical offline inputs:

```text
globalPlanner/inputs/
├── heightmap00.npz
└── wp00_191.csv
```

### Waypoint CSV

Currently contains grid coordinates:

```text
121,24
133,58
91,92
53,82
27,64
...
```

The final waypoint normally represents the return to the starting point.

---

# Running the Planner

Build only this package:

```bash
cd ~/Simulation_ws

colcon build --symlink-install \
    --packages-select dstar_navigation

source install/setup.bash
```

Run:

```bash
ros2 launch dstar_navigation offline_planner.launch.py \
    waypoints_file:=wp00_191.csv \
    heightmap_file:=heightmap00.npz
```

The planner should start:

```text
dstar_node
offline_sequence_planner.py
static_transform_publisher
rviz2
```

---

# Coordinate Frames

There are currently two different map definitions.

### Waypoint-generator maps

```text
resolution = 0.25 m/cell
origin = (-20.9503, -14.1658)
shape = (114,169)
```

### Planner heightmap

```text
resolution = 0.10 m/cell
origin = (-4.5525, -10.3392)
shape = (274,419)
```

Therefore, waypoint grid coordinates should **not be assumed to be planner grid coordinates**.

The intended solution is:

```text
Waypoint Grid
      ↓
Grid → World
      ↓
Common World Frame
      ↓
World → Planner Grid
      ↓
D*
```

A coordinate bridge/resampling step still needs to be implemented and validated.

---

# Current Status

| Component                | Status               |
| ------------------------ | -------------------- |
| D* node                  | Working              |
| Offline sequence planner | Working              |
| Waypoint loading         | Working              |
| Heightmap loading        | Working              |
| Costmap generation       | Working              |
| Sequential D* requests   | Working              |
| RViz visualization       | Working              |
| Map/waypoint alignment   | **Needs correction** |
| Coordinate bridge        | **Pending**          |

The current `Start or goal is outside global boundaries` error is expected to be investigated as a **map coordinate-frame mismatch** before modifying the D* algorithm itself.
