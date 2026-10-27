# Adaptive Pure Pursuit — Nav2 MPPI Benchmark Bench

This stack runs the adaptive pure-pursuit controller against a mock test
orchestrator, feeds the generated path and odometry into Nav2 through a bridge,
and records the full run for diagnostics and reporting.

## What the new launch does

The main entrypoint is now the MPPI launch file:

```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py
```

It starts the following nodes together:

- `controller_server` — Nav2 controller server using the MPPI controller plugin.
- `lifecycle_manager_navigation` — lifecycle manager for the controller server.
- `nav2_bench_bridge` — bridges `/path` and `/odometry/filtered` into Nav2's `FollowPath` action interface.
- `test_orchestrator` — publishes synthetic paths, odometry, and the current case ID.
- `control_monitor` — subscribes to path/odom/cmd topics and writes the CSV/report metrics.
- `rviz_visualizer` — overlays the planned path and actual trajectory for inspection.
- `robot_state_publisher` and `rviz2` — publish the URDF and visualize the robot state.

## Files

| File | Role |
|---|---|
| `launch/mppi_control_nodes.launch.py` | Main entry point — starts Nav2, the bridge, orchestrator, metrics node, RViz, and rosbag recording. |
| `adaptive_pure_pursuit/nav2_bench_bridge.py` | Receives `/path` and `/odometry/filtered`, republished odometry on `/odom`, and sends each path as a `FollowPath` action goal to Nav2. |
| `adaptive_pure_pursuit/test_orchestrator.py` | Publishes synthetic `/path`, `/odometry/filtered`, `/testing/current_case`, and consumes `/cmd_vel` from the controller. |
| `adaptive_pure_pursuit/control_monitor.py` | Subscribes to path/odom/cmd_vel/case topics, logs CSV metrics, and writes the HTML diagnostics report on shutdown. |
| `generate_report.py` *(external script)* | Produces the PDF-style report referenced by the shutdown chain. |

## Running it

```bash
# Build once (or after any Python edit, if not using --symlink-install)
colcon build --symlink-install
source install/setup.bash
```

### Run all 12 built-in test cases (default)
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py
```

### Run one or more specific test cases
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py test:=TC-03
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py test:=TC-03,TC-07,TC-10
```
An unknown ID (for example `TC-99`) logs a clear error and aborts the run cleanly.

### Run a custom path instead
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py \
    custom_path:="0,0;1,0;2,0.5;3,1"
```
Format: semicolon-separated `x,y` waypoint pairs, at least 2 points.
Runs as a single case tagged `CUSTOM` in the report.
**`custom_path` always takes priority over `test`** if both are supplied.

### Skip rosbag recording
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py record_bag:=false
```

## Launch arguments

| Argument | Default | Meaning |
|---|---|---|
| `test` | `''` (all) | Empty = run all 12 cases. Otherwise a single ID or comma-separated list, e.g. `TC-01,TC-05`. |
| `custom_path` | `''` (unused) | `x,y;x,y;...` waypoint string. Takes priority over `test` if set. |
| `record_bag` | `true` | Whether to record `/path`, `/odometry/filtered`, and `/cmd_vel` to a rosbag under `/tmp`. |

## Data flow

The current setup is:

1. `test_orchestrator` publishes a synthetic path and odometry stream.
2. `nav2_bench_bridge` subscribes to those topics and forwards the path to Nav2 as a `FollowPath` action goal.
3. Nav2's `controller_server` computes `cmd_vel` commands.
4. The orchestrator consumes `/cmd_vel` to update the mock robot state and continue the benchmark.
5. `control_monitor` records the resulting behavior to CSV and produces the HTML report.

## Built-in test cases (`TC-01` … `TC-12`)

| ID | Scenario | Duration |
|---|---|---|
| TC-01 | Sine-wave path | 40 s |
| TC-02 | Step lane change (sudden lateral jump at x=5) | 40 s |
| TC-03 | Path running in the negative-x direction (behind the robot's start) | 40 s |
| TC-04 | Short static case, vehicle kinematics frozen — exercises the in-place turn / goal-behind-robot branch | 5 s |
| TC-05 | Arctangent-shaped curve | 40 s |
| TC-06 | Straight line, robot starts offset with a 45° heading error | 40 s |
| TC-07 | Higher-frequency sine wave | 40 s |
| TC-08 | Straight line with injected odometry noise | 10 s |
| TC-09 | Straight + circular arc + straight (U-turn shape) | 40 s |
| TC-10 | Straight line with a "phantom obstacle" that shifts the path mid-run (~4 s in) | 40 s |
| TC-11 | Figure-eight (Lissajous) path | 40 s |
| TC-12 | Long multi-segment path: straight, arc, straight, arc, straight | 40 s |

Each case ends when the target is reached or when the effective deadline is hit. The effective deadline is the larger of the configured `global_timeout_s` and the case-specific duration.

## Reports

Two reports are produced automatically when the run finishes, regardless of
whether you ran all tests, specific tests, or a custom path:

- **HTML** — `<csv_output_path>_report.html` (default `/tmp/mppi_controller_metrics_report.html`)
  Generated by `control_monitor.py` on shutdown. Contains:
  - Overall pass/fail banner and whole-system statistics.
  - A per-test breakdown table at the bottom, one row per case actually run (or a single `CUSTOM` row), with lateral error, overshoot, jerk, and pass/fail metrics.
  - Opens automatically in your default browser.

- **PDF** — `/tmp/controller_report.pdf`
  Generated by the separate `generate_report.py` script referenced in the launch file.

The CSV data itself lives at `csv_output_path` (default `/tmp/mppi_controller_metrics.csv`) and now includes a `case_id` column tagging every sample.

## Troubleshooting

- **Launch fails with a missing Nav2 package** — the launch file now falls back to the standard Nav2 bringup parameters if `navstack_nav2` is not present in the active overlay. If you want the custom Nav2 config, make sure that package is available and sourced before launching.
- **Report file missing or stale after a run** — almost always means either:
  1. You edited source but didn't rebuild: `colcon build --symlink-install && source install/setup.bash`.
  2. `control_monitor` crashed on startup — check the terminal for a `Traceback` from that process.
- **Browser shows old content** — `webbrowser.open()` reopens the same `file://` URL each run; hard-refresh (`Ctrl+Shift+R`) or close/reopen the tab.
- **Invalid `test` ID** — logged as an error (`Unknown test ID(s): ...`), run aborts cleanly; check spelling/case (`TC-01`, not `tc1` or `1`).
- **`custom_path` rejected** — must be `x,y;x,y;...` with at least 2 points and valid floats; the exact malformed segment is named in the error log.
 
- **test orchestrator over-publishing /path issue
   the orchestrator republishes the current path on every state update instead of only on a few lifecycle events, and
   the bridge ignores repeated path messages that are effectively identical, so it doesn’t keep reissuing the same goal.

###If the HTML report didn't automatically pop open in your browser at the end of the run, you can launch it manually from your terminal using either of these commands:

```bash
wslview /tmp/mppi_controller_metrics_report.html

```

Or, using the Windows explorer executable directly:

```bash
explorer.exe $(wslpath -w /tmp/mppi_controller_metrics_report.html)

```

## Obstacle scenario generation (new)

This release adds a built-in obstacle scenario generator and preserves the original static `costmap.npz` workflow. Use the `use_costmap` and `scenario` launch arguments to control behavior:

- Baseline (no obstacles, unchanged):
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py test:=TC-04
```

- Obstacle-aware (enable costmap and use a generated scenario):
```bash
ros2 launch adaptive_pure_pursuit mppi_control_nodes.launch.py test:=TC-04 use_costmap:=true scenario:=blocking_obstacle
```

Available scenarios: `empty`, `single_obstacle`, `blocking_obstacle`, `random_obstacles`, `corridor`. XXXXXX

Notes:
- If `use_costmap:=true` and `scenario` is empty (default), the original `costmap.npz` publisher is used (unchanged).
- The generator publishes a `nav_msgs/OccupancyGrid` on `/local_costmap/costmap` (Transient Local QoS), compatible with MPPI critics.
- Use the `path` launch argument (or `custom_path`) to provide an external mission path file or inline path string.

The `control_monitor` now includes obstacle-aware metrics (`collision`, `min_clearance_m`) in the CSV and HTML report.


