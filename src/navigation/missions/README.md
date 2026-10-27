# 🗺️ ROAR 2026 — Competition Mission Datasets & Ground Truth Assets

[![Module](https://img.shields.io/badge/Module-navigation-E10600?logo=ros&logoColor=white)](file:///home/draaven/ROAR_2026/src/navigation)
[![Dataset](https://img.shields.io/badge/Datasets-Worlds_1__3-00D9FF)](file:///home/draaven/ROAR_2026/src/navigation/missions)

The `missions` directory houses the ground truth terrain data, obstacle fields, and pre-computed waypoint missions for Worlds 1, 2, and 3.

---

## 📁 Directory Structure

```text
src/navigation/missions/
├── mission_1/                        # Competition World 1 Dataset
│   ├── assets/                       # Ground truth map and obstacle assets
│   │   ├── costmap.npz               # 2.5D Traversability cost matrix
│   │   ├── heightmap.npz             # 2.5D Elevation grid & origin coordinates
│   │   ├── obstacle_data.npy         # Array of rock coordinates, radii, and bounding boxes
│   │   └── aruco_data.yaml           # Ground truth survey 3D coordinates for all ArUco tags
│   └── waypoints/                    # Synthesized waypoint sequences (wp00 .. wp299)
│       ├── wp00/wp00.npy             # Array of [column, row] grid coordinates
│       └── ...
│
├── mission_2/                        # Competition World 2 Dataset (same structure)
└── mission_3/                        # Competition World 3 Dataset (same structure)
```

---

## 🔍 Data Formats & Loading Guide

### 1. Heightmap (`heightmap.npz`)
```python
import numpy as np

data = np.load("assets/heightmap.npz")
grid = data['grid']           # 2D float32 elevation array
origin_x = data['origin_x']   # Metric X coordinate of grid[0, 0] (-18.431 m)
origin_y = data['origin_y']   # Metric Y coordinate of grid[0, 0] (-7.687 m)
resolution = data['resolution'] # Grid cell size in meters (0.25 m)
```

### 2. Obstacle Data (`obstacle_data.npy`)
```python
obstacles = np.load("assets/obstacle_data.npy", allow_pickle=True)
# Each obstacle entry contains:
# {'name': 'Rock_1', 'x': 5.52, 'y': 2.92, 'z': 0.57, 'radius': 0.45}
```

### 3. Waypoint Sequences (`wp*.npy`)
```python
# Grid coordinates: list of [col, row]
wp_grid = np.load("waypoints/wp00/wp00.npy")

# Convert to metric world coordinates:
metric_x = origin_x + wp_grid[:, 0] * resolution
metric_y = origin_y + wp_grid[:, 1] * resolution
```

