# World Setup Pipeline Stages (`world_setup`)

This folder contains the separate generator stages and templates that comprise the world setup pipeline. Each generator folder is structured with its own `inputs/`, `outputs/`, and orchestrator `script.py` so they can be run **standalone** or folder-by-folder.

---

## 📁 Directory Structure Overview

```text
world_setup/
├── initial_inputs/        # Permanent templates (base world & heightmap)
│   ├── i_world/           # base Gazebo world file (.world)
│   └── i_heightmap/       # clean terrain elevation data (.npz)
├── obsData_gen/           # Stage 1: Obstacle data generation
├── world_gen/             # Stage 2: Fused world creation
├── heightMap_gen/         # Stage 3: Fused world heightmap baking
├── costMap_gen/           # Stage 4: Costmap & CSV computation
└── rocks_ws/              # Model mesh database (rock_1 to rock_9)
```

---

## ⚙️ Running Individual Folders Standalone

To run any folder independently, place the required input files in that stage's `inputs/` folder, navigate to the stage directory, and run its `script.py`. The script will search for inputs automatically and generate outputs in the `outputs/` subfolder.

---

### 1. Stage 1: Obstacle Data Generation (`obsData_gen`)
Samples rough terrain elevation to generate coordinate locations for rock placement.

- **Inputs**: Copy a `.npz` heightmap (from `initial_inputs/i_heightmap`) into `obsData_gen/inputs/`.
- **Command**:
  ```bash
  python3 script.py --density 0.012 --collidable-ratio 0.50
  ```
- **Outputs**: Generates `outputs/obstacle_data.npy`.

---

### 2. Stage 2: World Generation (`world_gen`)
Fuses the generated obstacle list into a clean template to create a Gazebo world with static rocks.

- **Inputs**:
  - Copy `obstacle_data.npy` (from `obsData_gen/outputs`) into `world_gen/inputs/`.
  - Copy the clean Gazebo `.world` template (from `initial_inputs/i_world`) into `world_gen/inputs/`.
- **Command**:
  ```bash
  python3 script.py
  ```
- **Outputs**: Generates fused `outputs/generated.world`.

---

### 3. Stage 3: Heightmap Generation (`heightMap_gen`)
Bakes a metric elevation grid `.npz` representing the heights of the combined terrain and fused rocks.

- **Inputs**: Copy the fused `.world` file (from `world_gen/outputs`) into `heightMap_gen/inputs/`.
- **Command**:
  ```bash
  python3 script.py --resolution 0.25
  ```
- **Outputs**: Generates `outputs/heightmap.npz` and a grayscale preview `outputs/heightmap.png`.

---

### 4. Stage 4: Costmap Generation (`costMap_gen`)
Converts the metric heightmap into traversability cost maps (slope and roughness components) and outputs CSVs.

- **Inputs**: Copy the baked `heightmap.npz` (from `heightMap_gen/outputs`) into `costMap_gen/inputs/`.
- **Command**:
  ```bash
  python3 script.py --gradient-scale 150.0 --stability-scale 90.0
  ```
- **Outputs**: Generates:
  - `outputs/costmap.npz` (and grayscale preview `outputs/costmap.png`)
  - `outputs/csv/` containing `total_cost.csv`, `cost_x.csv`, and `cost_y.csv`
