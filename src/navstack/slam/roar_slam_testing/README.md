# roar_slam_testing

Test harness that runs and **grades** the full SLAM module (localization +
mapping) on the teleoperated marsyard2024 simulation, against the team's
"ERC Marsyard Module-Based Benchmarking Criteria".

## What's inside

| File | What it is |
|---|---|
| `roar_slam_testing/slam_evaluator.py` | The grader: compares `/odom` to Gazebo ground truth live, computes every SLAM benchmark metric, writes CSVs + a final score report |
| `roar_slam_testing/obstacle_simulator.py` | Fake perception for the mapping module: publishes detections of virtual obstacles when the rover is in range + FOV |
| `config/scenario_marsyard.yaml` | The mission definition: start point, 4 waypoints, 6 virtual obstacles, stop-detection thresholds, benchmark targets |
| `roar_slam_testing/slam_visualizer.py` | Live 8-panel dashboard: path+waypoints+markers overlay, 3D orientation, error plot, ArUco timeline, FIS bars, occupancy map, scoreboard |
| `launch/slam_test.launch.py` | ONE command that launches the entire SLAM stack + harness (everything except Gazebo) |

## The test in one picture

```
    YOU (teleop)                    THE HARNESS GRADES
 ┌────────────────┐        ┌─────────────────────────────────┐
 │ drive:         │        │ pose error vs ground truth      │
 │ start → WP1 →  │        │ yaw error                       │
 │ WP2 → WP3 →    │  ───►  │ drift rate (cm/m, cm/min)       │
 │ WP4 → start    │        │ localization dropouts           │
 │                │        │ ArUco corrections applied       │
 │ STOP ~5 s at   │        │ ZUPT count                      │
 │ every waypoint │        │ waypoint-stop scoring           │
 └────────────────┘        │ map alignment + coverage        │
                           └─────────────────────────────────┘
```

## How to run the test (step by step)

**0. Build first** (see workspace README — message packages before everything).

**1. Terminal 1 — start the simulation** (sim team's launch, unmodified):
```bash
source /opt/ros/humble/setup.bash
cd ~/roar_simulation && source install/setup.bash
ros2 launch roar_simulation basic_rover_mars.launch.py
```
Wait for `Configured and activated diff_drive_controller`.

**2. Terminal 2 — start the whole SLAM stack + harness:**
```bash
source /opt/ros/humble/setup.bash
cd ~/roar_onsite_ws && source install/setup.bash
ros2 launch roar_slam_testing slam_test.launch.py
```
Startup lines you should see:
```
[ieskf_node]:       ArUco map: 10 markers loaded
[ieskf_node]:       Output   : /odom  (TF OFF)
[aruco_simulator]:  Loaded 10 ArUco markers ...
[obstacle_simulator]: Loaded 6 virtual obstacles ...
[slam_evaluator]:   Scenario: 4 waypoints, 6 obstacles
```

**2b. Terminal 3 — live dashboard (recommended):**
```bash
ros2 run roar_slam_testing slam_visualizer
```
Waypoints show as yellow ✕, ArUco markers as magenta ▪ — drive the
blue/green dot onto the yellow ✕ marks.

**3. Drive the mission** with your usual teleop (Terminal 4):
```
start (0,−2) → WP1 (6,4) → WP2 (7,21) → WP3 (−4,20) → WP4 (−5,2) → start
```
Rules of the run:
- **STOP for ~5 seconds** at every waypoint and at the final return to
  start. A stop is registered when ground-truth speed stays < 0.03 m/s for
  2 s within 2.5 m of the expected waypoint. Stops are when ZUPT fires and
  when the waypoint score is recorded — exactly like the real mission.
- Waypoints must be visited **in order** (the evaluator tracks "next
  expected target"; a stop far from it is ignored for scoring).
- Watch the live status line every 2 s:
```
err avg=0.041 max=0.113 m | yaw=0.82° | dist=23.4 m | aruco=57 zupt=812 | stops=2 next=WP3
```

**4. End the run — Ctrl+C in Terminal 2.** The evaluator writes to
`~/slam_eval_<timestamp>/`:

| File | Contents |
|---|---|
| `report.txt` | The full score table (also printed to the console) |
| `pose_error.csv` | Time series: t, distance traveled, pose error, yaw error |
| `waypoint_stops.csv` | Per-stop: GT distance to waypoint, filter distance to waypoint, filter-vs-GT error |

## Reading the report

```
SLAM = 0.30·PoseAcc + 0.15·YawAcc + 0.20·Drift + 0.20·MapAlign
     + 0.10·Continuity + 0.05·UpdateRate            pass ≥ 85/100
```

| Metric | Target | Whose responsibility |
|---|---|---|
| pose error avg | ≤ 0.30 m | localization |
| yaw error avg | ≤ 5° | localization |
| drift rate | ≈ ≤ 1 cm per m | localization |
| dropouts | 0 | localization |
| ArUco utilization | informational (% of detections actually applied) | localization |
| map alignment | ≤ 0.20 m | mapping |
| obstacle coverage | informational (% of obstacles that made it into the map) | mapping |

Sub-score mapping (transparent, documented in code): 100 at ≤ 0.5× target,
linear down to 0 at ≥ 2× target.

Three waypoint numbers per stop, and what they mean:
- `gt_dist_to_target` — how well the DRIVER stopped on the waypoint
  (driver skill, not SLAM's problem)
- `filter_dist_to_target` — what a judge reading OUR estimate would score
- `filter_vs_gt_error` — pure localization error at the stop (this is ours)

## Scenario variations

Copy `scenario_marsyard.yaml`, edit, and point the launch at it
(`scenario_file` param on both `obstacle_simulator` and `slam_evaluator`).
Things to vary between runs:
- waypoint positions / order (different route difficulty)
- ArUco noise (`noise_base`, `noise_per_meter` on `aruco_simulator`) or
  range (`max_distance`) — degraded-perception scenarios
- disable ArUco entirely (`aruco_enabled: False` on `ieskf_node`) — measures
  how much the markers are actually buying you (do this once; the delta in
  pose error is the ArUco contribution)
- obstacle count / placement — mapping stress test

## Notes & known simplifications

1. **Virtual obstacles.** The marsyard world has no obstacle models. The 6
   obstacles exist only as fake detections for the mapping test; the rover
   can physically drive through them.
2. **Obstacle detections are published in the `map` frame** on purpose —
   `active_mapping` short-circuits TF when frames match, avoiding the
   missing `map→odom` TF in sim. Real-rover TF wiring is a perception
   integration task.
3. **`use_sim_time` is False everywhere** — matches the validated runs
   (Gazebo at real-time factor 1.0, wall-clock dt).
4. **ArUco range capped at 8 m** — the sim tags are 24 cm; a real detector
   wouldn't resolve them much farther.
5. The evaluator detects stops from **ground truth** speed, so driver
   behaviour can't fake localization quality.
