# D* Lite Navigation — Testing Suite Documentation

This directory contains the testing suite designed to verify and benchmark the `dstar_navigation` package across static, dynamic, hybrid, rough terrain, and full ERC (European Rover Challenge) simulation scenarios.

All test runs automatically execute the planning node, generate test environments, monitor goal completion, perform path checks (connectivity, collision-free), and save the outputs to `test/results/`.

---

## Output Files
For every test scenario run, the suite generates files in `test/results/`:
1. **Path CSV (`test/results/<scenario>_path_v<version>.csv`)**: Contains the list of planned path coordinates in the map coordinate system.
   - Format: `x,y`
   - Real-world coordinates (meters) matching the map frame.
2. **Path Plot (`test/results/<scenario>_plot_v<version>.png`)**: A high-resolution matplotlib visualization overlaying the planned path on top of the costmap grid, marking the start, goal, robot position, and coordinates.

---

## Phase 1 — Static Map Planning (Correctness)

### Objective
Verify that the D* Lite global planner successfully computes optimal, collision-free paths in static environments of varying geometry and handles impossible (enclosed) scenarios gracefully.

### Scenarios
1. **`open`**: A clear flat map to verify optimal near-diagonal planning.
2. **`wall`**: A single rectangular barrier blocks the direct path between start `(0.5, 0.5)` and goal `(4.5, 4.5)`.
3. **`u_trap`**: The robot starts inside a U-shaped wall and must navigate out to a goal outside the U-shape.
4. **`corridor`**: A narrow corridor (3-4 cells wide) is the only connection. Assesses clearance penalty steering.
5. **`enclosed`**: The start area is completely walled in. Verifies the planner aborts with failure when no path exists.

### How to Run
To run all static tests sequentially:
```bash
python3 test/run_static_tests.py
```
To run a specific scenario manually using launch file:
```bash
ros2 launch dstar_navigation test_static.launch.py scenario:=u_trap
```

---

## Phase 2 — Dynamic Obstacle Updates (Clear Map)

### Objective
Verify the correctness of D* Lite's **incremental replanning** algorithm (`km_` accumulation, vertex updating) when obstacles appear dynamically along the robot's pre-calculated path.

### Scenarios
1. **`surprise_wall`**: Starts with a clear map. Once the robot moves 1 meter towards the goal, a wall is injected across its path.
2. **`sequential_walls`**: Three separate walls appear one by one at different locations as the robot advances.
3. **`simulated_lidar`**: Map is initially fully unknown (`NO_INFORMATION` / -1). As the robot moves, a circular 270° LiDAR range reveals obstacles and free cells within a 1.5m radius.

### How to Run
To run all dynamic tests sequentially:
```bash
python3 test/run_dynamic_tests.py
```
To run a specific scenario manually using launch file:
```bash
ros2 launch dstar_navigation test_dynamic.launch.py scenario:=surprise_wall
```

---

## Phase 3 — Hybrid Scenario (Flat Terrain)

### Objective
Validate navigation in flat environments containing both pre-known static maps and runtime dynamic obstacles, as well as handling goal changes mid-flight.

### Scenarios
1. **`corridor_block`**: A horizontal corridor is pre-known. A dynamic barrier blocks the corridor at $x=60$ mid-flight. The planner detours by exiting the left side and going around the outside of the corridor.
2. **`goal_change_dynamic`**: The robot is traveling to Goal 1 `(4.5, 2.5)`. Halfway through, a wall blocks the path and a new Goal 2 `(4.5, 0.5)` is issued. The planner successfully preempts the active goal, cancels the previous planning thread, and redirects the robot.

### How to Run
To run all hybrid tests sequentially:
```bash
python3 test/run_hybrid_tests.py
```
To run a specific scenario manually using launch file:
```bash
ros2 launch dstar_navigation test_hybrid.launch.py scenario:=corridor_block
```

---

## Phase 4 — Rough Terrain (No Additional Obstacles)

### Objective
Verify that the planner handles continuous cost distributions representing slope and terrain roughness (as output by the heightmap converter) rather than just binary free/lethal grids.

### Scenarios
1. **`rough_patch`**: A central high-cost terrain block (cost 60). The planner routes *around* the patch using the free spaces.
2. **`gradient_slope`**: Cost increases linearly with Y (higher elevation = higher cost). The path curves towards lower Y regions, hugging the bottom valley to minimize traversal cost.

### How to Run
To run all terrain tests sequentially:
```bash
python3 test/run_terrain_tests.py
```
To run a specific scenario manually using launch file:
```bash
ros2 launch dstar_navigation test_terrain.launch.py scenario:=gradient_slope
```

---

## Phase 5 — Full ERC Simulation (Canyons + Dynamic Barriers)

### Objective
Perform an end-to-end integration test representing a full ERC traverse: navigating through a large-scale heightmap with canyon obstacles and continuous roughness while dynamically evading surprise obstacles.

### Scenario
- **`erc_simulation`**: A 400x400 (20m x 20m) grid containing canyon networks and gravel zones. A dynamic canyon collapse blocks the middle opening once the robot drives past the halfway point, forcing D* Lite to run an incremental update on the 160,000-cell grid.

### How to Run
To run the ERC simulation:
```bash
python3 test/erc_simulation.py
```
To run manually using launch file:
```bash
ros2 launch dstar_navigation test_erc.launch.py
```

---

## Phase 6 — Near-Goal Fallback & Multi-Waypoint

### Near-Goal Fallback (LiDAR style unreachable goal)

#### Objective
Test the BFS closest-reachable-pose fallback when the goal becomes dynamically blocked by obstacles revealed via simulated LiDAR scanning.

#### Scenario
- **`lidar_unreachable_goal`**: The goal `(4.5, 4.5)` is surrounded by a hidden 9×9 lethal obstacle box. As the robot approaches within LiDAR range, the map is updated and the planner snaps the goal to the nearest free cell within 5.0m. The test verifies the rover safely stops at the fallback pose rather than driving into the obstacle.

#### How to Run
```bash
python3 test/run_fallback_test.py
```
To run manually:
```bash
ros2 launch dstar_navigation test_fallback.launch.py
```

---

### Multi-Waypoint Navigation on Complex ERC Map

#### Objective
Verify sequential waypoint navigation across the full 400×400 ERC map (20m × 20m) with canyon walls, gravel fields, and dynamic barriers.

#### Scenario
- **`multi_waypoint`**: The rover starts at `(1.0, 1.0)` and visits 4 waypoints: `(18.0, 3.0)` → `(18.0, 18.0)` → `(2.0, 18.0)` → `(2.0, 2.0)`. Each waypoint is accepted, planned, and confirmed as reached before the next is dispatched.

#### How to Run
```bash
python3 test/run_waypoints_test.py
```
To run manually:
```bash
ros2 launch dstar_navigation test_waypoints.launch.py
```

---

## Module Bringup (Standalone Runs)

These are **not test scenarios** — they bring up individual modules for interactive development or integration with other team packages (APF, APP, SLAM, etc.).

```bash
# Global planner standalone (from workspace root)
./run_global.sh
./run_global.sh map_scenario:=lidar_maze_120 start_x:=1.0 start_y:=1.0

# Local controller standalone (MPPI, subscribes to /global_plan)
./run_local.sh

# Full integrated stack
./run_both.sh
```

Or use the interactive menu:
```bash
cd empty_test && python3 test_menu.py
# Select b1, b2, or b3 from the MODULE BRINGUP section
```

