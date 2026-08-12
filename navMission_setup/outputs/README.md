# Generated Mars Yard Datasets (`outputs`)

This directory contains the final datasets and generation scripts used to create the simulated Mars Yard environments for the **ERC 2026 Navigation Mission**.

Each execution of the pipeline generates multiple density variants of the world (e.g., `world_1`, `world_2`, `world_3`). These generated files are then automatically synchronized directly into the `dev_environment/worlds/` directory, allowing them to be loaded immediately.

---

## 🚀 How to Generate the Worlds

We have provided a master script that flawlessly automates the entire generation process:
- Procedurally generating rocks with mathematical static physics.
- Generating Heightmaps and Costmaps.
- Calculating the specific `Z` elevations for 15 ArUco markers based on the mission coordinate spreadsheet.
- Fusing the rocks and the ArUco markers perfectly into the Gazebo `.world` files.
- Synchronizing everything to the `dev_environment`.

### Step 1: Run the Master Pipeline
From the root of your simulation workspace, run the following:
```bash
cd navMission_setup/
./build_full_worlds.sh
```

### Step 2: Launch the World!
Once the generator finishes (it only takes a minute), you can immediately launch the fully integrated simulation directly from the `dev_environment`:
```bash
cd ../dev_environment/
./launch_test.sh full world1 rviz
```
*(You can replace `world1` with `world2` or `world3` to test different obstacle densities).*

---

## 🛡️ Built-In Collision & Generation Safeguards

The generation pipeline has been deeply customized to prevent physics engine explosions and guarantee flawless rover deployment:

1. **Static Mathematical Placements:** Rocks are generated statically using exact pitch/roll calculations instead of relying on Gazebo's dynamic physics engine to drop and settle them, significantly increasing simulation performance and preventing rover jitter.
2. **Rover Spawn Exclusion Zone:** The rock generator has a strict 2.5-meter exclusion radius hardcoded around `(0,0)` to guarantee the rover never spawns intersecting a rock, which previously caused the rover to flip violently onto its back.
3. **Gentle Rover Drops:** The `basic_rover.launch.py` has been updated to spawn the rover safely at `Z=0.5m` (down from `Z=2.5m`) to ensure smooth terrain landings without harsh suspension bounce.
4. **ArUco Marker Exclusion Zones:** The precise `(X, Y)` coordinates of all 15 ArUco markers have been injected directly into the rock generator. A strict 1.0-meter exclusion radius prevents any procedural rocks from spawning on top of, or blocking, the mission markers.

---

## 📁 Dataset Folder Contents

Each generated dataset folder contains the following files:

| File Name | Description |
| :--- | :--- |
| `world{index}.world` | Fused Gazebo world description containing the terrain, ArUco markers, and rock model assets. |
| `world{index}.launch.py` | ROS 2 launch script configured to load the fused world file in Gazebo. |
| `obstacle_data.npy` | NumPy binary coordinate array containing the `(x, y, z)` pose and size data of all spawned obstacles. |
| `obstacle_data_info.txt` | Statistics and parameters used to generate the obstacles. |
| `heightmap.npz` | Elevation matrix file representing the final combined terrain and obstacles. |
| `heightmap.png` | Grayscale visual representation of the elevation grid. |
| `costmap.npz` | Combined slope and roughness cost matrix used for traversability cost modeling. |
| `costmap.png` | Grayscale visual representation of the cost mapping. |
| `csv/` | Folder containing total, slope-based (X), and roughness-based (Y) cost grids in `.csv` format. |
| `metadata.txt` | Detailed logging of parameters, dates, bounds, and output paths of the run. |
