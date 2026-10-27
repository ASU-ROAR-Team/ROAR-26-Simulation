# 🚀 USE_ME: Navigation Mission Workspace How-To Guide

Welcome! This `USE_ME.md` guide provides step-by-step instructions for launching the Mission Control GUI, previewing mission maps, rendering waypoints with Y-inversion coordinate math, and executing navigation evaluation runs.

---

## 🎛️ Step 1: Launch the Mission Control GUI

To open the master PyQt GUI orchestrator:

```bash
cd ROAR-26-Simulation/src
./start_gui.sh
```

---

## 🖼️ Step 2: Render & Preview Mission Waypoints (`RENDER MAP`)

To preview waypoints overlaid on terrain elevation heatmaps, safe pebbles, and collidable rock obstacles:

1. In the GUI, select your target **WORLD** (World 1, 2, or 3) and **WP INDEX** (e.g. `WP INDEX: 2`).
2. Click **RENDER MAP**.

### What happens behind the scenes:
- The GUI purges old preview images from `/tmp/preview_mission_gui.png`.
- It executes `preview_mission.py --world <W> --wp <WP> --save /tmp/preview_mission_gui.png`.
- `preview_mission.py` reads `costmap.npz` and `wp<WP>.npy`, applies Y-inversion metric math ($Y = H - 1 - row$), plots the rover start position, obstacles, and path, and saves a fresh PNG image.
- The GUI instantly updates the visual display panel with the new preview map.

### Standalone Command Line Preview:
You can also generate preview maps directly from the terminal without opening the GUI:

```bash
python3 src/navigation/scripts/preview_mission.py --world 1 --wp 0 --save /tmp/preview_w1_wp0.png
xdg-open /tmp/preview_w1_wp0.png
```

---

## 🚀 Step 3: Run Navigation Missions & Referee Evaluation

### Option A: Via Mission Control GUI (Recommended)
1. In `asu_roar_gui.py`, navigate to the **ORCHESTRATION & BATCH QUEUE** tab.
2. Select your desired World and Waypoint set.
3. Click **➕ ADD TO BATCH QUEUE**.
4. Click **RUN BATCH PIPELINE**.
5. The GUI will orchestrate launching Gazebo simulation, the Nav2 stack, `erc_waypoint_follower.py`, and `referee_engine` headlessly.
6. Upon completion, the interactive HTML evaluation report opens automatically in your browser:
   `${ROAR_WS}/reports/master_report_*.html`

### Option B: Manual Terminal Execution
Follow the 4-step sequence in individual terminal windows:
1. **Terminal 1 (Simulation)**: `bash launch_test.sh noise world1`
2. **Terminal 2 (Nav2 Stack)**: `ros2 launch navstack_nav2 roar_nav2_only.launch.py`
3. **Terminal 3 (Referee Engine)**: `ros2 launch referee_engine referee.launch.py`
4. **Terminal 4 (Waypoint Follower)**:
   ```bash
   python3 ${ROAR_WS}/src/navstack/path-planning/globalPlanner/scripts/erc_waypoint_follower.py --auto --params ${ROAR_WS}/src/navstack/navstack_nav2/config/roar_nav2_sim.yaml
   ```

---

## 🧪 Step 4: Run Controller Benchmarks (`bench_marking/`)

To run MPPI / Adaptive Pure Pursuit controller benchmarks:

```bash
# Run built-in test suite (TC-01 to TC-12)
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py

# Run specific test case with obstacle costmap scenario
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py test:=TC-04 use_costmap:=true scenario:=blocking_obstacle
```

---

## 🔗 Related Documentation
- 🎛️ [Navigation Mission Workspace README](./README.md)
- 🌍 [Simulation Workspace USE_ME Guide](${ROAR_WS}/src/roar2026/USE_ME.md)
- 🧭 [ROS 2 Navigation Stack README](../navstack/README.md)
