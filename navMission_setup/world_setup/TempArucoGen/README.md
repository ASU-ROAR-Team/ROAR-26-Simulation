# ArUco Generation and Fusion Pipeline (Self-Contained)

This directory contains a fully self-contained pipeline to generate heightmaps and costmaps, calculate offsets, map coordinates, and fuse custom models into the horizontal horizontally-oriented simulation world.

---

## 1. Directory Structure

* **`scripts/`**: Contains the execution scripts (including local copies of `heightmap_generator.py` and `costmap_generator.py`).
* **`heightmap/`**: Output directory for generated terrain heightmaps.
* **`costmap/`**: Output directory for generated costmaps.
* **`world/`**: Output directory for the fused world file (`world_Rotated_Aruco.world`).
* **`aruco_data/`**: Output directory for the generated numpy `aruco_data.npy` coordinates matrix of shape `(15, 3)`.
* **`models/`**: Place your custom model directories (such as `aruco_1`, `aruco_2`, etc.) in this folder.
* **`run_all.sh`**: Master shell script.

---

## 2. How to Run the Pipeline

Simply run the master shell script from the root of this directory:
```bash
bash run_all.sh
```
This script will:
1. Delete any old generated files to ensure a clean start.
2. Run heightmap and costmap extraction locally.
3. Apply coordinate mapping and lookup heights.
4. Inject marker models.
5. Copy files to active packages and compile the workspace using `colcon build`.

---

## 3. How to Configure the Pipeline

### A. Changing the Origin Coordinate translation Offset:
Open [scripts/step2_generate_npy.py](scripts/step2_generate_npy.py). At the very top under **`CONFIGURATION PARAMETERS`**, edit:
```python
OFFSET_X = -16.0
OFFSET_Y = -6.0
```

### B. Customizing Landmark Coordinates:
Open [scripts/step2_generate_npy.py](scripts/step2_generate_npy.py). Edit the values inside the **`LANDMARKS`** list (under section 2) directly to change the $X$ and $Y$ coordinates.

### C. Mapping a Specific Model to a Landmark:
Open [scripts/step3_fuse_world.py](scripts/step3_fuse_world.py). Under **`LANDMARK-TO-MODEL MAPPING CONFIGURATION`**, edit the dictionary:
```python
LANDMARK_MODELS = {
    1: "aruco_1",          # Landmark 1 (L1) loads model://aruco_1
    2: "aruco_2",          # Landmark 2 (L2) loads model://aruco_2
    3: "aruco_placeholder", # Landmark 3 (L3) loads model://aruco_placeholder
    # ...
}
```
You can assign any directory name located inside the `models/` folder to any landmark index.
