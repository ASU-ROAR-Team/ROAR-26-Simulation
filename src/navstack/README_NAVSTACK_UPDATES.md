# NavStack 2026 Integration Updates

This document summarizes all the recent updates, parameter tuning, and custom nodes added to the `navStack_2026` repository to make it fully compatible with the new automated ROAR-26 Marsyard Simulation & Evaluation pipeline.

---

## 1. Offline Planning & Heightmap Configuration (`config.yaml`)
To ensure the offline global planner accurately understands the physical constraints of the rover on the simulated Marsyard terrain, a new configuration file was introduced (`path-planning/globalPlanner/offline_tools/config.yaml`) with highly tuned baseline parameters:

* **`heightmap.resolution`: `0.0501274`** (Tuned from the default grid sizes to precisely match the metric scale of the Gazebo terrain mesh).
* **`costmap.resolution`: `0.0501274`** (Synchronized with the heightmap).
* **`costmap.max_slope`: `25.0` degrees** (Any terrain steeper than 25 degrees is now considered a lethal obstacle).
* **`costmap.max_height_diff`: `0.40` meters** (Steps higher than 0.4m between cells are marked as lethal to prevent the rover from attempting impossible rock climbs).

## 2. Robust Waypoint Follower (`erc_waypoint_follower.py`)
In automated batch testing, a stuck rover would previously hang the entire evaluation pipeline forever. We heavily modified `path-planning/globalPlanner/scripts/erc_waypoint_follower.py`:

* **Timeout Logic:** The follower now continuously monitors the rover's progress toward the active waypoint.
* **Auto-Skip:** If the rover is unable to reach the waypoint within a calculated timeout window (because it got stuck on terrain or caught in a local costmap minima), the script automatically aborts the current goal and commands Nav2 to proceed to the next waypoint in the list.

## 3. Nav2 Trajectory Tuning (`roar_nav2_sim.yaml`)
To adapt the navigation stack to the rough terrain and avoid extreme oscillation near waypoints, several parameters were aggressively tuned in the controller server:

* **Goal Tolerances (Controller):** Adjusted to be slightly more forgiving, allowing the rover to register a "success" even if it cannot perfectly center on a waypoint due to uneven rocks.
  * `xy_goal_tolerance`: `0.25` ➡️ **`0.35`**
  * `yaw_goal_tolerance`: `0.15` ➡️ **`0.25`**
* **Path Follower Critics (MPPI/DWB):** Tuned to generate smoother velocity commands and force tighter adherence to the global path around dense rock clusters.
  * `PathAlignCritic.cost_weight`: `2.0` ➡️ **`3.0`**
  * `PathAlignCritic.max_path_occupancy_ratio`: `0.05` ➡️ **`0.95`**
  * `PathFollowCritic.cost_weight`: `5.0` ➡️ **`10.0`**
  * `PathAngleCritic.max_angle_to_furthest`: `1.0` ➡️ **`1.5`**
  * `PathAngleCritic.threshold_to_consider`: `0.5` ➡️ **`0.35`**

## 4. Topic Name Synchronization (`fixed.md`)
* Addressed an issue where the `ros_gz_bridge` was searching for the ground truth pose on `/world/marsyard/pose/info` instead of `/world/rover_world/pose/info` (the default SDF world name used in the `dev_environment`).
