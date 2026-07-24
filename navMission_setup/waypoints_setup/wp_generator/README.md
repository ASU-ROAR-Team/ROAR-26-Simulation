# Waypoint Generator Pipeline (`wp_generator`)

## 🤖 Overview
The **Waypoint Generator** is a robust procedural mission-planning module designed for the **ROAR-26 Simulation Pipeline**  It ingests environment costmaps and raw obstacle streams, samples collision-free spatial coordinates, builds multi-point mission paths, evaluates their traversal difficulty, and exports structured trajectory sets ready for ROS 2 and Nav2 integration.

---

## 📂 Project Architecture & File Structure

```text
wp_generator/
├── inputs/
│   ├── costmap.npz             # 2D environmental cost grid matrix
│   ├── obstacle_data.npy       # Raw obstacle dictionaries (coordinates & flags)
│   ├── rover_config.json       # Physical rover dimensions and specs
│   └── wp_constraints.json     # Pipeline bounds, ratios, and generation settings
├── outputs/
│   ├── difficulty.npy          # 1D array of calculated difficulty scores per mission
│   ├── generation_log.json     # Execution metadata and configuration snapshot
│   └── missions.npy            # 3D array of exported trajectories shape (10, 5, 2)
├── read_outputs.py             # CLI utility script to parse and print binary outputs
├── visualize_missions.py       # Interactive Matplotlib UI window to flip through missions
└── wp_generator.py             # Core pipeline execution script

```
### File Responsibilities:

* **`wp_generator.py`**: The core execution engine. Handles input loading, dynamic ratio mapping, Poisson Disk Sampling, path building, constraint validation, difficulty scoring, and file exports.
* **`inputs/costmap.npz`**: Stores the navigational cost grid representing safe vs. hazardous terrain.
* **`inputs/obstacle_data.npy`**: Contains raw object logs (rocks, obstacles) including spatial coordinates and behavior flags.
* **`inputs/rover_config.json`**: Defines the rover’s physical footprint parameters (`rover_length_m`, `rover_width_m`, clearance margins, etc.).
* **`inputs/wp_constraints.json`**: Houses primary constraints like target mission counts, spacing ratios, and margin rules.
* **`outputs/missions.npy`**: Binary 3D NumPy array containing the generated mission coordinates `[x, y]` sorted by difficulty.
* **`outputs/difficulty.npy`**: Binary 1D NumPy array tracking the calculated multi-factor difficulty scores for each exported mission set.
* **`outputs/generation_log.json`**: Records execution metadata, runtime logs, timestamp stamps, and a full snapshot of the active constraints used during that specific build run.
* **`read_outputs.py`**: A helper script to unpack and display binary `.npy` contents directly in the terminal text stream.
* **`visualize_missions.py`**: An interactive GUI viewer built with Matplotlib widgets and keyboard event handlers to visually inspect trajectories over the costmap.
---


## 🪨 Obstacle Data & Sizing Mechanics

* **Collidability:** The obstacle dataset is structured as a list of dictionaries containing coordinates and flags. The script uses defensive programming (`obj.get("is_collidable", True)`) to safely check whether an object is flagged as an active hazard (`True`) or drivable background terrain (`False`).
* **Spatial Footprints (`rock_radius_cells`):** Obstacles are stamped onto the costmap using a programmatic radius footprint:
```python
rock_radius_cells = max(1, int(round(0.5 / res)))
#in wp_generator line 121
```
* *What it does:* It takes a flat radius value (e.g., `0.5` meters) and divides it by your grid resolution (`res`) to calculate how many grid cells the obstacle occupies.
* *How to edit:* You can modify the `0.5` value to a larger or smaller meter scale, or update this block to dynamically query physical `.stl` mesh bounding extents (`mesh.extents`) for precise rock scaling.
---


## 📏 Constraints & Dynamic Percentage-Driven Scaling

To ensure the generator scales cleanly across different map sizes (from tiny test grids to massive simulation worlds), boundary margins and spacing rules are calculated **dynamically via percentages**:


---
#
### 📏 Constraints Breakdown (`wp_constraints.json`)

* **`candidate_count` (500):** Max raw spatial points sampled via Poisson Disk Sampling before filtering.
* **`max_cost_threshold` (0.8):** Max allowable terrain cost for valid waypoint placement.
* **`min_spacing_cells` / `max_spacing_cells` (~1.21 – ~4.23):** Dynamic distance limits between consecutive waypoints.
* **`boundary_margin_cells` (1):** Strict inner buffer preventing waypoints from spawning on map edges.
* **`duplicate_distance_threshold` (3.0):** Spatial threshold to cluster and remove redundant paths.
* **`target_mission_count` (10):** Final number of unique mission paths exported.
* **`max_attempts` (1000):** Path-building iteration limit to prevent infinite loops.
* **`seed` (42):** Random seed for reproducible procedural generation.
* **Ratio Settings (`boundary_margin_ratio: 0.05`, `min_spacing_ratio: 0.1`, `max_spacing_ratio: 0.35`):** Percentage rules scaling boundaries and spacing relative to map dimensions and diagonal length.
---

* **`boundary_margin_cells`**: Dynamically derived from a percentage (`boundary_margin_ratio`, default 5%) of the map's smallest dimension, establishing a safe inner perimeter where waypoints cannot be spawned.
---

## 🎛️ What Else Can Be Tuned & Edited in the Generator?
1. **`candidate_count`**: Controls how many raw Poisson Disk samples are initially generated across the map. Increasing this gives the pipeline a larger pool of options.
2. **`max_cost_threshold`**: Sets the strictness of allowable terrain cost. Lowering this forces the rover to stick strictly to safer, lower-cost regions.
3. **`target_mission_count`**: Determines how many final unique mission paths are built and exported (default is `10`).
4. **`seed`**: The random number generator seed. Changing this allows you to generate entirely different procedural batches while keeping each run reproducible.
5. **Mission Path Length (Waypoint Count per Mission):** Inside the path builder logic, you can adjust how many sequential waypoints make up a single mission (e.g., configuring paths to have 5 waypoints vs. 10 waypoints).

---

## 📊 Difficulty Scoring Mechanics

Each generated mission path receives a multi-factor difficulty score derived from a weighted combination of:

1. **Total Path Length:** Cumulative Euclidean distance connecting all waypoints in the sequence.
2. **Path Curvature & Turn Angles:** Sum of sharp turns across the route (computed via vector dot products between consecutive path segments). Sharper zig-zags incur higher penalties.
3. **Terrain Cost:** The average and peak cost values encountered along the rasterized path coordinates.

---

## 🚀 How to Use & Run

### 1. Run the Main Generator Pipeline

Execute the core generation script to parse inputs and export fresh mission trajectories:

```bash
python3 wp_generator.py

```


### 2. Inspect Outputs in the Terminal

To view a human-readable text breakdown of the exported binary files, run:

```bash
python3 read_outputs.py

```

### 3. Launch the Interactive Visualizer

To view trajectories overlaid on top of the costmap using an interactive graphical interface with navigation buttons and keyboard shortcuts:

```bash
python3 visualize_missions.py

```

* **Controls:** Click the **Next** / **Previous** buttons on the window, or press your keyboard's **Right / Left arrow keys** (or keys `n` and `p`) to smoothly flip through all generated mission paths.

```

```