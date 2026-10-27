# D* Lite Standalone Global Planning Testing Environment

This guide explains how to build, launch, and test the **D* Lite Global Planner** in isolation without requiring Gazebo, Nav2 stack dependencies, or a physical robot.

---

## Architecture Overview

The standalone testing environment consists of 5 modular ROS 2 nodes running together:

```
                          ┌───────────────────────────┐
                          │       fake_slam.py        │
                          │ - Publishes /odom, TF     │
                          │ - Publishes /heightmap    │
                          │ - Subscribes to /cmd_vel  │
                          └─────────────┬─────────────┘
                                        │
           ┌────────────────────────────┼───────────────────────────┐
           │ (/heightmap)               │ (/odom)                   │
           ▼                            ▼                           │
┌──────────────────────────┐   ┌──────────────────────────┐         │
│  heightmap_to_costmap.py │   │       dstar_node         │         │
│ (Heightmap + Inflation)  │   │   (Standalone D* Lite)   │         │
└──────────┬───────────────┘   └────────────┬─────────────┘         │
           │ (/unified_costmap)             │                       │
           └───────────────────────────────►│                       │
                                            │ (/global_plan)        │
                                            ▼                       │
                               ┌──────────────────────────┐         │
                               │     path_follower.py     │         │
                               │   (Pure Pursuit Motion)  │         │
                               └────────────┬─────────────┘         │
                                            │ (/cmd_vel)            │
                                            └───────────────────────┘
```

---

## Build Instructions

```bash
# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Navigate to workspace root
cd ${ROAR_WS}/src/navstack

# 3. Build the global planner package
colcon build --packages-select dstar_navigation

# 4. Source the built workspace
source install/setup.bash
```

---

## Running the Standalone Test

```bash
ros2 launch dstar_navigation dstar_test_standalone.launch.py
```

The simulated robot starts at **(x: 1.0, y: 1.0)** on a 20m × 20m grid.

---

## Sending Goals to D* Planner

### Via ROS 2 Action CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 18.0, y: 18.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Via RViz 2 GUI

Use the **"2D Goal Pose"** tool from the top toolbar.

---

## Changes Summary

### Refactored Architecture

| Before | After |
|--------|-------|
| D* subscribed to 4 topics (`/map`, `/heightmap`, `/heightmap_range`, `/terrain_costmap`) | D* subscribes to 1 topic (`/unified_costmap`) |
| `/map` used alongside heightmap | Pure heightmap pipeline (`/map` dropped — heightmap is sole source of truth) |
| No inflation layer in costmap fuser | **2-Stage Inflation Layer** added (`inscribed_radius` hard lethal, `inflation_radius` cost decay) |
| `edgeCost()` had 6 terms | `edgeCost()` has 2 terms (clearance_field + terrain/inflation cost) |

### Files Modified

| File | Change |
|------|--------|
| `scripts/heightmap_to_costmap.py` | **Rewritten** — pure heightmap fuser with 2D OpenCV distance-transform inflation layer |
| `src/dstar_planner.cpp` | **Simplified** — single subscription, 2-term edgeCost |
| `src/dstar_node.cpp` | **Simplified** — same changes as plugin version |
| `config/dstar_params.yaml` | **Updated** — new parameter set |
| `navstack_nav2/config/roar_nav2_sim.yaml` | **Updated** — D* + heightmap_to_costmap inflation params |
| `CMakeLists.txt` | **Updated** — added fake_slam.py, path_follower.py install targets |

### Files Created

| File | Purpose |
|------|---------|
| `dstar_navigation/fake_slam.py` | Simulated SLAM with TF, odom, heightmap, robot marker |
| `dstar_navigation/path_follower.py` | Simple pure-pursuit path follower |
| `launch/dstar_test_standalone.launch.py` | Standalone test launch file |
| `UNIFIED_COSTMAP_REFACTOR.md` | Detailed changelog and tuning guide |
| `Global_planning_testing.md` | This file |

---

## Inflation Layer Pipeline

`heightmap_to_costmap.py` performs inflation using OpenCV's Euclidean distance transform (`cv2.distanceTransform`):

1. **Heightmap Lethal Detection**: Detects steep slopes ($> \text{MAX\_SAFE\_SLOPE}$) and height steps ($> \text{max\_traversable\_height\_diff}$).
2. **Inscribed Inflation** (`inscribed_radius`, default 0.25m): Any cell within this distance of a lethal cell is marked **hard lethal (100)** to account for the robot's physical footprint.
3. **Gradual Inflation Band** (`inflation_radius`, default 0.8m): Cells between `inscribed_radius` and `inflation_radius` receive exponentially decaying inflation costs:
   $$\text{inflation\_cost} = 99 \times \exp\left(-\text{scaling} \times \frac{d - r_{\text{inscribed}}}{r_{\text{inflation}} - r_{\text{inscribed}}}\right)$$
4. **Fusion**: Non-lethal cell cost is `max(terrain_cost, inflation_cost)`.

---

## Bug Fixes Applied During Code Review

### Critical Fixes

| Bug | Location | Fix |
|-----|----------|-----|
| **NaN Poisoning** — All heightmap cells being -1 caused `np.mean()` to return NaN, poisoning Sobel/Laplacian filters | `heightmap_to_costmap.py` line 149 | Early return publishing all-unknown costmap when no valid heightmap cells exist |
| **Clearance Field seeding from NO_INFORMATION** — Unknown cells (255) passed the `>= LETHAL_OBSTACLE (254)` check, creating false obstacle repulsion around unmapped areas | `dstar_planner.cpp` L531, `dstar_node.cpp` L712 | Added `&& costs[i] != NO_INFORMATION` guard |
| **Data Size Validation** — No check that `msg->data.size()` matched `width * height`, risking segfault on malformed messages | Both C++ files, costmap callback | Added size check with early return and warning log |
| **Negative Value Sanitization** — Negative OccupancyGrid values (other than -1) cast to `unsigned char` would wrap to large values | Both C++ files, costmap callback | `std::max(0, val)` before scaling |
| **Race Condition on `map_changed_`** — `execute_plan()` thread read/wrote `map_changed_` without holding `map_mutex_`, while the callback wrote it under the lock | `dstar_node.cpp` L263, L317 | Wrapped `map_changed_` access in `map_mutex_` lock guards |
| **Reference Copy Bug** — `hmap_msg.info = grid_msg.info` shared the object by reference, causing mutation side effects | `fake_slam.py` L103 | Replaced with field-by-field copy |
| **Uncapped Angular Velocity** — Heading error up to ±π × gain=2.0 could produce 6.28 rad/s angular commands | `path_follower.py` L88 | Clamped to ±1.5 rad/s |

---

## Fallback Sequences in D* Planner

The D* Lite planner contains a layered fallback system to handle various failure modes:

### 1. Start Pose Snapping (5×5 Neighborhood Search)
**Trigger:** Robot's starting cell is inside a lethal or invalid cell.
**Behavior:** Searches a 5×5 grid around start pose for nearest valid cell.

### 2. Goal Pose Snapping (BFS, 5.0m Radius)
**Trigger:** Goal cell is inside a lethal or invalid cell.
**Behavior:** Executes a BFS expanding outward up to 5.0m to find closest traversable cell.

### 3. D* Lite Incremental Replanning
**Trigger:** Costmap changes detected, goal unchanged.
**Behavior:** Incrementally updates only affected vertices via `updateVertex()` (100–700× faster).

### 4. A* Extraction Fallback
**Trigger:** Path extraction cycle detected during gradient descent.
**Behavior:** Executes forward A* search from stuck cell to goal using the same cost function.

### 5. Heightmap Range Fallback (Python)
**Trigger:** `/active_map/heightmap_range` topic not received yet.
**Behavior:** Falls back to `occupancy_heightmap_scale` (default 0.015).

### 6. All-Unknown Heightmap Fallback (Python)
**Trigger:** Every heightmap cell is -1.
**Behavior:** Short-circuits pipeline and publishes all-unknown costmap (`-1`).

---

## Parameter Tuning Reference

### D* Planner Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `clearance_field_weight` | `3.0` | Obstacle repulsion strength. ↑ = wider berth |
| `clearance_decay_distance` | `0.5` | Repulsion falloff (meters). ↓ = sharper |
| `terrain_weight` | `2.0` | Terrain difficulty penalty. ↑ = strongly prefer flat |
| `unknown_traversal_cost` | `2.0` | Cost multiplier for unknown cells |
| `heuristic_weight` | `1.0` | **Must be 1.0** for D* Lite correctness |

### Unified Costmap & Inflation Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `MAX_SAFE_SLOPE` | `0.5` | Slope threshold for normalization / hard-block |
| `MAX_SAFE_ROUGHNESS` | `0.5` | Roughness threshold for normalization |
| `steepness_weight` | `0.5` | 1st derivative weight in merged cost |
| `roughness_weight` | `0.5` | 2nd derivative weight in merged cost |
| `max_traversable_height_diff` | `0.30` | Height step > this (m) = lethal |
| `inscribed_radius` | `0.25` | Hard lethal inflation radius around obstacles (m) |
| `inflation_radius` | `0.8` | Soft cost decay buffer radius around obstacles (m) |
| `inflation_cost_scaling` | `3.0` | Decay rate of inflation cost buffer |
