# SIMULATION GUIDE — Running the SLAM Test

The exact, validated procedure for running the full SLAM test on the
marsyard2024 teleoperated simulation. Follow it in order. Every item in the
troubleshooting table at the bottom is a failure we actually hit and solved.

---

## 0. One-time setup

### Workspaces on the machine

| Workspace | Path | What's in it |
|---|---|---|
| Marsyard sim | `~/ROAR-26-Simulation/` | marsyard world + roar_rover model + diff_drive (sim team's — NEVER modified) |
| SLAM | `~/roar_onsite_ws/` | the 5 SLAM packages (this repo's `slam/` folder) |

### First-time build

```bash
# Sim workspace (once, or after sim team updates)
cd ~/ROAR-26-Simulation
colcon build
```

```bash
# SLAM workspace — MESSAGE PACKAGES FIRST
source /opt/ros/humble/setup.bash
cd ~/roar_onsite_ws
colcon build --packages-select roar_msgs active_mapping_msgs
source install/setup.bash
colcon build --packages-select roar_localization active_mapping roar_slam_testing
source install/setup.bash
```

⚠️ Make sure packages sit directly in `~/roar_onsite_ws/src/<package>/` —
NOT nested as `src/src/<package>` (happens if you unzip an archive that
already contains a `src/` folder).

---

## 1. Run order — 4 terminals

Order matters: the sim must be publishing before the SLAM stack starts.

### Terminal 1 — Marsyard simulation

```bash
source /opt/ros/humble/setup.bash
source ~/ROAR-26-Simulation/install/setup.bash
ros2 launch roar_simulation basic_rover_mars.launch.py
```

⚠️ `basic_rover_mars.launch.py` — the `_mars` matters. Plain
`basic_rover.launch.py` loads the EMPTY world (flat plane, no map, wrong
topic names) and nothing downstream will work.

Wait for BOTH:
- Gazebo window showing the Mars terrain WITH the rover on it
- Console line: `Configured and activated diff_drive_controller`

Quick sanity (scratch terminal):
```bash
ign topic -l | grep pose        # must show /world/marsyard/pose/info
```

### Terminal 2 — SLAM stack (one command)

```bash
source /opt/ros/humble/setup.bash
source ~/roar_onsite_ws/install/setup.bash
ros2 launch roar_slam_testing slam_test.launch.py
```

Verify the startup lines say **marsyard** (not empty):
```
[synthetic_imu_node]: /world/marsyard/pose/info[11] -> /imu/data
[ground_truth_bridge]: /world/marsyard/pose/info -> /zed2i/...
[ieskf_node]: ArUco map: 10 markers loaded
[ieskf_node]: Output   : /odom  (TF OFF)
[aruco_simulator]: Loaded 10 ArUco markers
[obstacle_simulator]: Loaded 6 virtual obstacles
[slam_evaluator]: Scenario: 4 waypoints, 6 obstacles
```

Then confirm data is flowing (scratch terminal):
```bash
ros2 topic hz /world/marsyard/pose/info   # ~20-60 Hz depending on machine
ros2 topic hz /imu/data                   # similar
ros2 topic hz /odom                       # similar — THE key check
```

### Terminal 3 — Live dashboard

```bash
source /opt/ros/humble/setup.bash
source ~/roar_onsite_ws/install/setup.bash
ros2 run roar_slam_testing slam_visualizer
```

8-panel window opens. Waypoints = yellow ✕, start = white ★, ArUco
markers = magenta ▪, obstacles = grey ▫. Blue = filter, green = ground
truth.

### Terminal 4 — Teleop

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

Keys: `i` forward, `,` reverse, `j`/`l` turn, `k` stop. The terminal must
have keyboard focus.

---

## 2. The mission

```
start (0,−2) → WP1 (6,4) → WP2 (7,21) → WP3 (−4,20) → WP4 (−5,2) → start
```

Rules:
- Navigate by the dashboard: drive the blue/green dot onto each yellow ✕.
- **STOP ~5 s at every waypoint** and at the final return. A stop registers
  when ground-truth speed < 0.03 m/s for 2 s within 2.5 m of the expected
  (in-order) waypoint. Stops fire ZUPT and record the waypoint score.
- Random pauses elsewhere are harmless (ZUPT still fires; no score entry).

Health checks while driving:
- Standing still at start: error should read ~0–2 cm. If it reads ~20 cm,
  the camera extrinsic is wrongly non-zero (see troubleshooting).
- Occasional `Synthetic IMU: rejecting sample` warnings in Terminal 2 are
  NORMAL on slow machines — that's the stutter guard working.

## 3. Ending the run

Ctrl+C **Terminal 2**. The evaluator writes to `~/slam_eval_<timestamp>/`:
- `report.txt` — full score table (also printed to console)
- `pose_error.csv`, `waypoint_stops.csv`

Score: `0.30·Pose + 0.15·Yaw + 0.20·Drift + 0.20·Map + 0.10·Cont + 0.05·Rate`,
pass ≥ 85.

---

## 4. Troubleshooting (every entry happened for real)

| Symptom | Cause | Fix |
|---|---|---|
| Gazebo opens EMPTY flat world, no map | launched `basic_rover.launch.py` | use `basic_rover_mars.launch.py` |
| Gazebo opens marsyard WITHOUT rover | launched `erc2025_remote_sim startsimulation.launch.py` (Husarion template leftovers) | use `roar_simulation basic_rover_mars.launch.py` |
| `file ... not found in share directory` | wrong workspace sourced — the marsyard launch lives in `~/ROAR-26-Simulation`, not `~/roar_simulation` | `source ~/ROAR-26-Simulation/install/setup.bash` |
| `/odom` never publishes | SLAM stack listening to the wrong GT topic (empty vs marsyard) | check Terminal 2 startup lines say `marsyard`; fix `GT_TOPIC` in `slam_test.launch.py`, rebuild |
| `/world/marsyard/pose/info` missing from `ros2 topic list` | bridge not running (Terminal 2 down) or Gazebo dead | `ign topic -l \| grep pose` to check Gazebo side; restart in order |
| BOTH `/world/empty/...` AND `/world/marsyard/...` present | two Gazebo instances alive | kill everything: `pkill -f gz; pkill -f ros2`, restart in order |
| `ros2 topic list` throws `xmlrpc ... !rclpy.ok()` | stuck ROS daemon | `ros2 daemon stop && ros2 daemon start` |
| Error pinned at ~20 cm while standing still | `cam_to_base_x/z` non-zero in sim (fake ZED topic carries the BASE pose, not a camera pose) | all `cam_to_base_*` = 0.0 in sim launches (already the default in this repo). Real rover uses measured values. |
| Error spikes to tens of meters, blue path rockets off | Gazebo RTF 10–45% on slow machines → pose stream stutters → differentiation blow-up | fixed in code (synthetic IMU guards + IESKF velocity clamp). Close heavy apps to raise RTF. |
| Colcon: `SyntaxError '(' was never closed` in setup.py | broken manual edit | restore `setup.py` from repo |
| Teleop `termios.error` | teleop run in a broken/reused terminal | open a fresh terminal for teleop |
| Rebuild "succeeds" but launch behaves old | edited source but rebuilt the wrong package, or forgot to re-source | rebuild the right package, `source install/setup.bash`, RESTART the launch |
| `bash: ...ros2_lab_ws...: No such file` on every terminal | stale line in `~/.bashrc` | `sed -i '/ros2_lab_ws/d' ~/.bashrc` |

---

## 5. Scenario variations

Edit `roar_slam_testing/config/scenario_marsyard.yaml` (waypoints,
obstacles, thresholds) or override per-node params in
`slam_test.launch.py`:
- ArUco difficulty: `noise_base`, `noise_per_meter`, `max_distance` on
  `aruco_simulator`
- ArUco OFF (measure its contribution): `aruco_enabled: False` on
  `ieskf_node` — run the same lap and compare reports
- Obstacle layout: `obstacles:` list in the scenario YAML
