# Navigation Mission Setup (`navMission_setup`)

This package provides a automated pipeline for generating parameterized Mars Yard environments for simulation testing. It takes a clean base world and elevation map, generates random obstacle configurations (rocks), fuses them into a standalone Gazebo world, and computes corresponding metric heightmaps and costmaps.

---

## 🗺️ Pipeline Flow Overview

The orchestrator script `add_world.py` cleans the temporary stage directories, runs each step of the pipeline sequentially, and collects all final outputs into a structured folder.

```mermaid
graph TD
    A[0. Initial Inputs] -->|Copy Base World & Heightmap| B(Stage 1: Obstacle Gen)
    B -->|Generate Placements| C[obstacle_data.npy]
    C -->|Copy to World Gen inputs| D(Stage 2: World Fusion)
    A -->|Copy Base World| D
    D -->|Fuse World| E[world{index}.world]
    E -->|Copy to Heightmap Gen inputs| F(Stage 3: Heightmap Gen)
    F -->|Generate elevation NPZ| G[heightmap.npz]
    G -->|Copy to Costmap Gen inputs| H(Stage 4: Costmap Gen)
    H -->|Calculate slope & roughness| I[costmap.npz & CSVs]
    I -->|Gather all outputs| J[outputs/world{index}/]
    E -->|Gather all outputs| J
    G -->|Gather all outputs| J
    C -->|Gather all outputs| J
```

---

## 🚀 How to Run the Pipeline

Run the orchestrator script `add_world.py` using Python 3, passing the desired rock density, collidable percentage, and output world index:

```bash
python3 add_world.py <density> <percentage> <index> [options]
```

### Parameters
- **`density`** (float): Target rock density in rocks per square meter (e.g. `0.012` for ~8 rocks in a 704m² area).
- **`percentage`** (float): Percentage of rocks that are solid/collidable (e.g. `50` for 50%). Values > 1.0 are automatically converted to a decimal ratio.
- **`index`** (int): Unique integer suffix for the generated dataset folder (e.g. `1` to output to `world1`).

### Optional CLI Arguments
- `--name NAME`: Custom name for the world dataset (defaults to `world{index}`).
- `--deadends`: Form barrier walls of rocks to create dead-ends in terrain.
- `--heightmap-resolution RESOLUTION`: Cell resolution of the generated heightmap in meters (default: `0.25`).
- `--gradient-scale SCALE`: Slope/gradient scale factor for cost mapping (default: `150.0`).
- `--stability-scale SCALE`: Roughness/terrain stability scale factor for cost mapping (default: `90.0`).

### Example Command
```bash
python3 add_world.py 0.025 60 1
```

---

## 📁 Master Outputs Directory Layout

Upon successful execution, a folder named `outputs/world{index}` is created containing:
```text
outputs/world{index}/
├── world{index}.world         # Standalone Gazebo world with fused rocks
├── obstacle_data.npy          # NumPy coordinates of placed rocks
├── obstacle_data_info.txt     # Placed rock statistics & parameters
├── heightmap.npz              # Metric heightmap elevation matrix
├── heightmap.png              # Grayscale elevation visualization
├── costmap.npz                # Terrain traversability cost grid
├── costmap.png                # Grayscale cost map visualization
├── csv/                       # Folder containing cost grid CSVs:
│   ├── total_cost.csv         # Fused traversability costs
│   ├── cost_x.csv             # Slope cost component
│   └── cost_y.csv             # Roughness cost component
└── metadata.txt               # Dataset metadata log
```

---

## 🧹 Cleaning Behavior
Each pipeline run automatically clears all files in the temporary `inputs/` and `outputs/` stage directories under `world_setup/` to guarantee a clean, isolated state. The `initial_inputs/` folder containing base world templates is **never** modified or touched.
