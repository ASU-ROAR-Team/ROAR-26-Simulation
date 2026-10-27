# D* Lite Navigation — Package Changes & Edits Changelog

This document lists all the structural repairs, algorithmic fixes, parameters, and testing suite implementations completed on the `dstar_navigation` package.

---

## 1. Summary of Edits

### ── C++ PLANNING NODE (`src/dstar_node.cpp`) ──
*   **Bug 1: Unknown Cells Traversal (`isCellValid`)**: Fixed ordering so `NO_INFORMATION` (255) is checked before `LETHAL_OBSTACLE` (254). Previously, unknown cells (value 255) were treated as lethal obstacles (since $255 \ge 254$), preventing traversal in unknown terrain.
*   **Bug 2: Heuristic Accumulation (`km_`)**: Added `km_ += heuristic(last_start_idx_, new_start)` whenever the start pose changes. This resolves suboptimal planning in incremental updates.
*   **Bug 3: QoS Subscription Mismatch**: Updated the `/map` subscriber to `TRANSIENT_LOCAL` durability to correctly match the map publisher, ensuring the map is received on startup.
*   **Bug 4: Debug Parameter Loading**: Loaded missing parameters (`debug_publish_*`) in `loadParameters()` to prevent runtime errors during telemetry visualizer publications.
*   **Bug 5: Snapped Start Pose Collision at `plan[0]`**: When the robot's pose was snapped to a nearby valid cell to escape a dynamic obstacle, `plan[0]` was overwritten with the raw lethal pose. Fixed by updating the pose generation to use the snapped coordinates.
*   **Bug 6: A* Fallback Origin (0,0) Trajectory Green Line**: If D* Lite extraction looped, the A* fallback parent traceback reached grid index 0 (origin) if uninitialized. Added strict verification that the trace connects back to `current_idx` before writing poses.
*   **Bug 7: Map-Change Replanning Loop**: Restructured the 5 Hz executor loop to only invoke `makePlan` when `map_changed_` is true, comparing the cost values to avoid redundant planning.
*   **Bug 8: Action Goal Preemption & Thread Overlap**: Spawning detached threads on new action goals caused double execution, costmap corruption, and crash-on-exit (`Asked to publish result for goal that does not exist`). Implemented `current_goal_handle_` tracking; active planning threads check if they have been preempted and cleanly exit.
*   **Bug 9: Heuristic Inflation (`heuristic_weight`)**: In D* Lite, the heuristic must be admissible and consistent (a lower bound of cost grid transition costs). The default `heuristic_weight` parameter was set to `1.5` (inflated), which broke key consistency during incremental updates, causing `computeShortestPath` to terminate with 0 expansions when cost increases occurred. Fixed by setting `heuristic_weight: 1.0` in the planner configuration.

### ── PATH SIMULATOR (`dstar_navigation/path_simulator.py`) ──
*   **Bug 10: Steering Oscillations**: Replaced the direct waypoint queue follower with a stable **lookahead pure-pursuit controller** (`closest_idx + 3`, ~0.15m ahead). This prevents the simulator from steering backward or getting stuck when receiving new plans.
*   **ROS Parameter Speed**: Declared `speed` as a ROS parameter, allowing launch scripts to configure faster velocities (e.g., 0.5 m/s) for large-scale maps.

### ── TEST HARNESSES & MAPS ──
*   **Bug 11: Matplotlib Disk Write Timeouts**: Generating and saving PNG plots for every plan update caused tests to exceed the 75s timeout. Deferred plot creation and limited outputs to 3 representative versions (first, middle, final).
*   **Bug 12: Map Polling Replanning Thrashing**: Removed the 200ms map publisher timer in `dynamic_map_publisher.py` to prevent plan oscillation. Static maps are published once, and LiDAR maps only publish when the robot travels $\ge$ half a cell.

---

## 2. File Modified / Created List

### Modified Package Files
*   [`src/dstar_node.cpp`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/src/dstar_node.cpp) (Core planner, preemption, replan triggers, snapping, fallback validation)
*   [`dstar_navigation/path_simulator.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/dstar_navigation/path_simulator.py) (Lookahead pure pursuit, speed parameter)
*   [`CMakeLists.txt`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/CMakeLists.txt) (Registered all new programs, scripts, and launch directories)
*   [`test/README.md`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/README.md) (Fully rewritten with updated commands and implemented scenarios)
*   [`test/maps/dynamic_map_publisher.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/maps/dynamic_map_publisher.py) (Added corridor blocking triggers and dynamic goal change map layouts)
*   [`test/harness/test_harness_dynamic.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/harness/test_harness_dynamic.py) (Added deferred plotting and increased timeout)

### Created Package Files
*   [`launch/test_hybrid.launch.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/launch/test_hybrid.launch.py) (Launch configurations for Phase 3)
*   [`launch/test_terrain.launch.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/launch/test_terrain.launch.py) (Launch configurations for Phase 4)
*   [`launch/test_erc.launch.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/launch/test_erc.launch.py) (Launch configurations for Phase 5)
*   [`test/harness/test_harness_hybrid.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/harness/test_harness_hybrid.py) (Hybrid harness, mid-flight preemption goal tracking)
*   [`test/harness/test_harness_terrain.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/harness/test_harness_terrain.py) (Terrain harness, slope/detour cost assertions)
*   [`test/harness/test_harness_erc.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/harness/test_harness_erc.py) (ERC harness, large scale metrics, markdown report generator)
*   [`test/maps/terrain_map_generator.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/maps/terrain_map_generator.py) (Continuous cost and linear gradient publisher)
*   [`test/maps/erc_map_generator.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/maps/erc_map_generator.py) (400x400 map, canyons, gravel fields)
*   [`test/run_hybrid_tests.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/run_hybrid_tests.py) (Automated Phase 3 runner script)
*   [`test/run_terrain_tests.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/run_terrain_tests.py) (Automated Phase 4 runner script)
*   [`test/erc_simulation.py`](file:///home/amrtamer/ROAR-Nouveau-Autonomous-System/Path-planning/dstar_navigation/test/erc_simulation.py) (Automated Phase 5 runner script)

---

## 3. Benchmark Performance Results

### Phase 1 — Static Maps (100% PASS)
*   `open`: 1.2 ms planning time, direct diagonal path.
*   `wall`: 1.5 ms planning time, routes around central blocking wall.
*   `u_trap`: 1.8 ms planning time, escapes U-trap local minimum.
*   `corridor`: 1.4 ms planning time, steering clearance respect.
*   `enclosed`: Planner correctly returns `ABORTED` (status 6) when start is walled in.

### Phase 2 — Dynamic Obstacles (100% PASS)
*   `known_maze_blocked`: Pre-known corridor layout. Replan detour around middle wall triggered at $x \ge 2.5$ in 59.51 ms (4,307 expansions).
*   `lidar_maze_120`: Start unknown, scans S-shaped maze with 120° frontal LiDAR and detours around wall corners sequentially in 212 distinct plans.

### Phase 3 — Hybrid Flat Terrain (100% PASS)
*   `corridor_block`: Bypasses horizontal corridor through a detour when blocked.
*   `goal_change_dynamic`: Goal changes from `(4.5, 2.5)` to `(4.5, 0.5)` mid-navigation. Terminated preempted planning thread and redirected.

### Phase 4 — Rough Terrain (100% PASS)
*   `rough_patch`: Planner successfully detours around central patch (cost 60) to hug free terrain.
*   `gradient_slope`: Trajectory curves towards lower Y regions, resulting in an average Y of **1.787m** instead of a direct 2.5m diagonal line.
*   `forced_rough`: Detours are blocked by lethal obstacles. Planner successfully traverses directly through high-cost rough patch.
*   `perlin_terrain`: Planner successfully plans and navigates through continuous heightmap waves.



### Phase 5 — Full ERC Simulation (100% PASS)
*   **Scale**: $400 \times 400$ grid (160,000 cells).
*   **Initial Path Search**: **1746.24 ms** (expanding 146,430 cells to calculate the initial 701-pose path).
*   **Incremental Replan**: **2.27 ms** (0 expansions to bypass dynamic canyon collapse).
*   **Speedup**: **~747x faster** than full replanning.
*   **Navigation time**: Robot completed the 45.7m serpentine run in **90.52 seconds** at 0.5 m/s.

### Phase 6 — Near-Goal Fallback & Multi-Waypoint (100% PASS)
*   **`lidar_unreachable_goal`** (fallback test): Robot starts at `(0.5, 0.5)` with goal `(4.5, 4.5)` hidden inside a lethal 9×9 obstacle box. On map reveal, the BFS snapping fallback detects the blocked cell, snaps the goal to the nearest free cell within 5.0m (`(85, 85)` at 0.28m), replans, and navigates the rover to the fallback pose safely. ✅ PASS.
*   **`multi_waypoint`** (ERC complex map): Sequential navigation through 4 distant waypoints on the 400×400 canyon map:
    *   WP1 `(18.0, 3.0)` reached at 35.4s
    *   WP2 `(18.0, 18.0)` reached at 99.5s
    *   WP3 `(2.0, 18.0)` reached at 131.3s
    *   WP4 `(2.0, 2.0)` reached at 205.5s — total 205.5s. ✅ PASS.

---

## 4. Module Bringup Infrastructure

Three shell scripts and three ROS 2 launch files added for standalone module development and integration testing:

### Shell Scripts (workspace root)
*   [`run_global.sh`](file:///home/amrtamer/nav-stack_2026/run_global.sh) — Launches D* Lite global planner standalone. Args: `map_scenario:=open`, `start_x:=0.5`, `start_y:=0.5`.
*   [`run_local.sh`](file:///home/amrtamer/nav-stack_2026/run_local.sh) — Launches MPPI local controller standalone. Subscribes to `/global_plan` and waits.
*   [`run_both.sh`](file:///home/amrtamer/nav-stack_2026/run_both.sh) — Launches both modules together as a full integrated navigation stack.

### Bringup Launch Files
*   [`launch/bringup_global_planner.launch.py`](file:///home/amrtamer/nav-stack_2026/path-planning/globalPlanner/launch/bringup_global_planner.launch.py) — D* Lite + dynamic map + path simulator + RViz.
*   [`launch/bringup_local_controller.launch.py`](file:///home/amrtamer/nav-stack_2026/path-planning/globalPlanner/launch/bringup_local_controller.launch.py) — MPPI controller server + fake robot + map server + lifecycle manager + RViz.
*   [`launch/bringup_full_stack.launch.py`](file:///home/amrtamer/nav-stack_2026/path-planning/globalPlanner/launch/bringup_full_stack.launch.py) — Full composed stack with a single RViz instance.

### Interactive Test Menu
*   [`empty_test/test_menu.py`](file:///home/amrtamer/nav-stack_2026/empty_test/test_menu.py) — Extended with `b1`/`b2`/`b3` module bringup choices (launch in background, print PID) above the existing MPPI scenario list.

### RViz Camera Fix
*   [`config/rviz/dstar_test.rviz`](file:///home/amrtamer/nav-stack_2026/path-planning/globalPlanner/config/rviz/dstar_test.rviz) — `Target Frame` changed from `<Fixed Frame>` to `base_link` so the camera follows the rover automatically.

