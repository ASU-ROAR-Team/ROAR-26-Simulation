# Waypoint Generator Pipeline (`wayPoint_generator`)

## TO DO
1- get actual rover dimensions
2- Avoid dublicated scores

## 🤖 Overview
The **Waypoint Generator** is a procedural mission-planning module designed for the **ROAR-26 Simulation Pipeline**. It ingests environment costmaps and raw obstacle streams, samples collision-free spatial coordinates using obstacle clearance only, builds 5-point waypoint sets, scores their difficulty as the exact sum of terrain cost along the path, and exports one file per waypoint set ready for ROS 2 and Nav2 integration.

---

## 📂 Project Architecture & File Structure

```text
wayPoint_generator/
├── inputs/
│   ├── costmap.npz             # 2D environmental cost grid matrix
│   ├── obstacle_data.npy       # Raw obstacle dictionaries (coordinates & flags)
│   ├── rover_config.json       # Physical rover dimensions and specs
│   └── wp_constraints.json     # Pipeline bounds, ratios, and generation settings
├── outputs/
│   ├── wp00_<score>.npy         # (5, 2) array — easiest waypoint set, difficulty = sum of terrain cost at all 5 points
│   ├── wp01_<score>.npy         # next-easiest, and so on — one file per set, index 0 = easiest
│   ├── ...
│   └── generation_log.json     # Execution metadata, aggregate difficulty stats, and config snapshot
├── read_outputs.py             # CLI utility script to list and print all wp*.npy outputs
├── visualize_missions.py       # Interactive Matplotlib UI window to flip through waypoint sets
└── wp_generator.py              # Core pipeline execution script
```

### File Responsibilities:

* **`wp_generator.py`**: The core execution engine. Handles input loading, dynamic ratio mapping, Poisson Disk Sampling, obstacle-clearance filtering, path building, constraint validation, difficulty scoring, and per-set file export.
* **`inputs/costmap.npz`**: Stores the navigational cost grid representing safe vs. hazardous terrain. Negative values (e.g. `-1`) are treated as "unknown/unmapped" and excluded wherever cost is read.
* **`inputs/obstacle_data.npy`**: Contains raw object logs (rocks, obstacles) including spatial coordinates and collidability flags. This is the *only* input that drives candidate filtering now.
* **`inputs/rover_config.json`**: Defines the rover's physical footprint parameters (`rover_length_m`, `rover_width_m`, clearance margins, etc.) — used purely for the clearance-radius check.
* **`inputs/wp_constraints.json`**: Houses primary constraints like target waypoint-set count, spacing ratios, and margin rules.
* **`outputs/wp{index}_{score}.npy`**: One binary `(5, 2)` NumPy array per waypoint set. `{index}` is zero-padded (`00`, `01`, ...) and reflects difficulty rank — `wp00` is always the easiest. `{score}` is the rounded integer sum of terrain cost across all 5 points in that set.
* **`outputs/generation_log.json`**: Records execution metadata, timestamp, waypoint-set count, aggregate difficulty stats (min/max/mean across all exported sets), and a full snapshot of the constraints used during that run. Does **not** contain a per-set score list — per-set scores live only in each file's name.
* **`read_outputs.py`**: Globs and sorts all `wp*.npy` files by index, then prints each set's points and score to the terminal.
* **`visualize_missions.py`**: An interactive GUI viewer built with Matplotlib widgets and keyboard event handlers to visually inspect each waypoint set's trajectory over the costmap.

---

## 🪨 Obstacle Data & Sizing Mechanics

* **Collidability:** The obstacle dataset is structured as a list of dictionaries containing coordinates and flags. The script uses defensive programming (`obj.get("is_collidable", True)`) to safely check whether an object is flagged as an active hazard (`True`) or drivable background terrain (`False`).
* **Spatial Footprints (`rock_radius_cells`):** Obstacles are stamped onto the costmap using a programmatic radius footprint:
```python
rock_radius_cells = max(1, int(round(0.5 / res)))
# in wp_generator.py, InputLoader.load()
```
* *What it does:* It takes a flat radius value (e.g., `0.5` meters) and divides it by your grid resolution (`res`) to calculate how many grid cells the obstacle occupies.
* *How to edit:* You can modify the `0.5` value to a larger or smaller meter scale, or update this block to dynamically query physical `.stl` mesh bounding extents (`mesh.extents`) for precise rock scaling.

---

## 🚧 Candidate Filtering — Obstacle Clearance Only

`CandidateFilter` filters candidates purely on distance-to-nearest-obstacle (via a precomputed Euclidean distance transform on the `obstacles` grid). It does **not** consider costmap values at all — a candidate passes if it's far enough from any collidable obstacle to fit the rover's clearance radius, regardless of terrain cost at that location.

If the required clearance exceeds what's achievable anywhere on the map (e.g. the rover is too big for the map's dimensions), filtering returns zero candidates rather than silently falling back to a looser check.

---

## 📏 Constraints & Dynamic Percentage-Driven Scaling

To ensure the generator scales cleanly across different map sizes (from tiny test grids to massive simulation worlds), boundary margins and spacing rules are calculated **dynamically via percentages** relative to the loaded map's actual dimensions.

### 📏 Constraints Breakdown (`wp_constraints.json`)

* **`candidate_count` (500):** Max raw spatial points sampled via Poisson Disk Sampling before filtering.
* **`min_spacing_cells` / `max_spacing_cells`:** Dynamic distance limits between consecutive waypoints, computed from ratios below relative to map diagonal.
* **`boundary_margin_cells`:** Strict inner buffer preventing waypoints from spawning on map edges, computed from `boundary_margin_ratio` relative to the map's smallest dimension.
* **`duplicate_distance_threshold` (3.0):** Spatial threshold to cluster and remove redundant waypoint sets.
* **`target_wp_count` (10):** Final number of unique waypoint sets exported.
* **`max_attempts` (1000):** Path-building iteration limit to prevent infinite loops.
* **`seed` (42):** Random seed for reproducible procedural generation.
* **Ratio Settings (`boundary_margin_ratio`, `min_spacing_ratio`, `max_spacing_ratio`):** Percentage rules scaling boundaries and spacing relative to map dimensions and diagonal length.
* **`max_cost_ratio` / `max_cost_threshold`:** ⚠️ Currently **unused** — left over from a cost-based filtering step that has been removed (see Candidate Filtering section above). `max_cost_threshold` is still computed at load time but nothing reads it. Safe to ignore, remove, or repurpose later if cost-based filtering is reintroduced.

---

## 🎛️ What Else Can Be Tuned & Edited in the Generator?
1. **`candidate_count`**: Controls how many raw Poisson Disk samples are initially generated across the map. Increasing this gives the pipeline a larger pool of options.
2. **`target_wp_count`**: Determines how many final unique waypoint sets are built and exported (default is `10`). Note: this truncates to the *easiest* N sets after ranking — the hardest sets among the deduplicated pool are dropped, not sampled across the full difficulty spread.
3. **`seed`**: The random number generator seed. Changing this allows you to generate entirely different procedural batches while keeping each run reproducible.
4. **Rover clearance (`rover_config.json`)**: `rover_length_m`, `rover_width_m`, and `clearance_margin_m` directly control how large a clearance radius candidates must satisfy. If a map is too small for the real rover's footprint, filtering will legitimately return zero valid candidates — this is expected geometric behavior, not a bug.

---

## 📊 Difficulty Scoring Mechanics

Each generated waypoint set's difficulty score is the **exact sum of costmap terrain cost** at all 5 points in the set (starting point + wp1 + wp2 + wp3 + wp4). No weighting, path length, or turn-angle factors are involved — this replaced an earlier multi-factor weighted score.

Waypoint sets are sorted ascending by this score before export, so `wp00` is always the lowest-cost (easiest) set, `wp01` the next, and so on.

---

## 🚀 How to Use & Run

### 1. Run the Main Generator Pipeline

Execute the core generation script to parse inputs and export fresh waypoint sets:

```bash
python3 wp_generator.py
```

### 2. Inspect Outputs in the Terminal

To view a human-readable text breakdown of all exported waypoint files, run:

```bash
python3 read_outputs.py
```

### 3. Launch the Interactive Visualizer

To view waypoint trajectories overlaid on top of the costmap using an interactive graphical interface with navigation buttons and keyboard shortcuts:

```bash
python3 visualize_missions.py
```

* **Controls:** Click the **Next** / **Previous** buttons on the window, or press your keyboard's **Right / Left arrow keys** (or keys `n` and `p`) to flip through all generated waypoint sets. Press **Ctrl+C** in the terminal to close the window.