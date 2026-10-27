# roar_localization

IESKF-based localization for the ASU ROAR rover (ERC 2026). Fuses:

- **BNO055 IMU** (NDOF mode — absolute orientation + gravity-free accel)
- **Wheel encoders** (one per side, via `/joint_states`)
- **ZED2i visual odometry** (position observations)
- **ArUco landmark detections** (absolute position fixes from known markers)

into a single odometry estimate on **`/odom`**, consumed by the mapping
module inside SLAM.

Based on Huang et al. 2024 (FIS-IESKF for WIVO), adapted for outdoor rough
terrain and this rover's sensor set.

---

## Architecture

```
                        ┌───────────────────────────────────────────┐
 /imu/data ────────────►│ imuCallback (~100 Hz)                     │
                        │   IMU mount correction (R_imu_to_body)    │
 /joint_states ────────►│   WIO  (encoders + IMU yaw → vx, dp)      │
                        │   WO-FIS (slip, ω, vx → k_Q)              │
                        │   IESKF.predict(a, R, dp, dt, k_Q)        │
                        │   ZUPT detector → updateVelocity(0)       │──► /odom
                        └───────────────────────────────────────────┘    /roar/ieskf_diagnostics
                        ┌───────────────────────────────────────────┐    TF odom→base_footprint
 /zed2i/.../pose ──────►│ zedPoseCallback (~30 Hz)                  │       (if publish_tf)
                        │   ZED extrinsic correction                │
                        │   WVO  (outlier reject → blend → window)  │
                        │   VO-FIS (confidence, jump → k_V)         │
                        │   IESKF.update(z, k_V)                    │
                        └───────────────────────────────────────────┘
                        ┌───────────────────────────────────────────┐
 /aruco/detections ────►│ arucoCallback (event-driven)              │
                        │   marker map lookup (aruco_map.yaml)      │
                        │   distance filter + noise scaling         │
                        │   IESKF.updateAruco(...)                  │
                        └───────────────────────────────────────────┘
```

### State vector (12D)

| Block | Meaning | Frame |
|---|---|---|
| `p` | position | world |
| `v` | velocity | world |
| `b_a` | accel bias | body |
| `b_w` | gyro bias | body |

Orientation and gravity are **not** in the state — the BNO055 NDOF fusion
owns orientation, and its linear acceleration output is already
gravity-free. This is the main adaptation vs. the paper's 18D state.

### Three observation types

| Method | Observes | H | Fired by |
|---|---|---|---|
| `update(z, k_V)` | position | `[I 0 0 0]` | ZED/WVO callback |
| `updateVelocity(v, σ)` | velocity | `[0 I 0 0]` | ZUPT when stationary |
| `updateAruco(...)` | position (from landmark geometry) | `[I 0 0 0]`, tight σ | ArUco callback |

All three share one iterated Kalman kernel (`applyUpdate` in `ieskf.hpp`)
using the standard IEKF (Bell & Cathey) iteration
`δx_{i+1} = K·(z − h(x⊕δx_i) + H·δx_i)` and a Joseph-form covariance
update. Linear observations converge in one iteration at full strength;
future nonlinear observations get proper Gauss-Newton behaviour.

### ZUPT (Zero Velocity Update)

When the rover is stationary its true velocity is exactly zero — a free
measurement. The `ZuptDetector` (`zupt.hpp`) requires ALL of:

1. |v_L| and |v_R| below `zupt_encoder_speed_threshold`
2. |ω_z| below `zupt_angular_velocity_threshold`
3. accel variance (ring buffer) below `zupt_accel_variance_threshold`

held continuously for `zupt_min_stationary_duration` before firing
`updateVelocity([0,0,0], σ_v_zupt)` every IMU tick while stationary.

**Why ZUPT is not optional in this architecture:** position propagates from
WIO dead-reckoning, not from integrating filter velocity. Consequence:
accel bias `b_a` is *unobservable from position updates* (verified by unit
test 3). Velocity observations — ZUPT — are the only strong observability
path for `b_a`. The mission's waypoint stops are when the filter actually
calibrates its bias.

### ArUco geometry

Given a detection of marker `id` at `p_marker_camera` and the marker's known
world position `p_marker_world`:

```
p_rover = p_marker_world − R_body_to_world · (t_cam_in_body + R_cam_to_body · p_marker_camera)
```

That implied rover position is applied as a position observation with
distance-scaled noise `σ = sigma_aruco_base + sigma_aruco_per_m · distance`.
Detections farther than `aruco_max_distance` are skipped. Unknown IDs
(not in the map) are skipped with a throttled warning.

The marker map is loaded once at startup from YAML (`aruco_map.hpp/cpp`):

```yaml
markers:
  - id: 0
    position: [0.0, -3.0, 0.55]
```

`config/aruco_map.yaml` currently holds the **real marsyard2024 simulation
positions** (IDs 0–9, tag center z = 0.55 m). For ERC, replace with IDs
51–64 from Update Report #3.

### Fuzzy Inference Systems (adaptive noise)

- **WO-FIS** (10 Sugeno rules): slip ratio, normalized ω_z, normalized vx →
  `k_Q` multiplier on the velocity process-noise block. Wheel slip →
  inflate Q → trust prediction less.
- **VO-FIS** (9 Sugeno rules): ZED confidence (from covariance trace),
  WVO jump ratio → `k_V` multiplier on observation noise. Bad tracking →
  inflate V → trust ZED less.

---

## Executables

| Executable | Purpose | Sim/Real |
|---|---|---|
| `ieskf_node` | The localization filter | both |
| `aruco_simulator_node` | Fakes ArUco perception from Gazebo ground truth (FOV + range + noise model) | sim only |
| `ground_truth_bridge` | Fakes the ZED pose topic from Gazebo ground truth | sim only |
| `synthetic_imu_node` | Fakes the BNO055 by differentiating Gazebo ground truth (EMA-smoothed, 30-sample warmup, plus min-dt / max-speed / max-accel guards against Gazebo RTF stutter) | sim only |

## Key topics

| Topic | Dir | Type | Notes |
|---|---|---|---|
| `/imu/data` | in | `sensor_msgs/Imu` | body-frame, gravity-free accel |
| `/joint_states` | in | `sensor_msgs/JointState` | `wheel_lhs_mid_joint` / `wheel_rhs_mid_joint` |
| `/zed2i/zed_node/pose_with_covariance` | in | `PoseWithCovarianceStamped` | |
| `/aruco/detections` | in | `roar_msgs/ArucoDetectionArray` | |
| `/odom` | out | `nav_msgs/Odometry` | THE deliverable — consumed by mapping |
| `/roar/ieskf_diagnostics` | out | `Float64MultiArray` | `[aruco_received, aruco_applied, zupt_fired, cov_trace_pos, k_Q, k_V]` at ~10 Hz |
| TF `odom → base_footprint` | out | | only if `publish_tf:=true` (FALSE in sim — diff_drive owns it) |

## Critical parameters (sim values in `launch/simulation.launch.py`)

| Param | Sim value | Why |
|---|---|---|
| `child_frame_id` | `base_footprint` | true X-forward body frame (REP-105 root); NOT `base_link`, which the URDF's `base_footprint_joint` yaws -90deg off of true forward |
| `imu_yaw_deg` | `-90.0` | the BNO055 is mounted on `base_link`, which is yawed -90deg from true forward — this rotates its orientation/accel/gyro into `base_footprint` before fusion, so WIO heading matches the ZED position track |
| `publish_tf` | `False` | diff_drive_controller already broadcasts odom→base_footprint |
| `pulses_per_rev` | `6.2831853` | Gazebo publishes cumulative **radians** (2π per rev) |
| `max_delta_pulses` | `50.0` | must survive the synthetic IMU's 0.5 s warmup gap |
| `wvo_window_size` / `wvo_min_fill` | `1` / `1` | zero-lag passthrough of the (perfect) sim ZED |
| `wvo_w_wo` | `0.0` | rocker-bogie WIO heading unreliable on flat ground — revisit on real terrain |
| `cam_to_base_x/y/z` | `0.0` | **sim only** — the fake ZED topic carries the BASE pose; non-zero values inject a constant error. Real rover: measured values |
| `max_velocity` | `5.0` | IESKF velocity clamp — bounds damage from garbage IMU samples (sim stutter or real sensor glitch) |
| `sigma_pos` | `1e-4` | sim ZED is ground truth |
| `aruco_max_distance` | `8.0` | sim tags are only 24 cm — realistic detector range |

Real-rover launch (`launch/ieskf_launch.py`) marks every value that must be
measured before first drive with `← MEASURE`.

## Unit tests

Six suites, pure C++ (no ROS needed):

```bash
g++ -std=c++17 -I include -I /usr/include/eigen3 -c src/ieskf.cpp -o /tmp/ieskf.o
g++ -std=c++17 -I include -I /usr/include/eigen3 test/test_ieskf.cpp /tmp/ieskf.o -o /tmp/test_ieskf
/tmp/test_ieskf
```

(same pattern for the others; `test_aruco` additionally links `-lyaml-cpp`)

| Suite | Covers | Tests |
|---|---|---|
| `test_math_utils` | skew, exp_so3, quat/euler conversions, angle wrap | 23 |
| `test_ieskf` | predict, all 3 update types, bias observability, covariance health, k_V scaling | 21 |
| `test_zupt` | all four gating conditions, timing, transitions, reset | 13 |
| `test_wio` | forward/yaw/pitch motion, spike rejection, v_L/v_R, reset | 20 |
| `test_wvo` | outlier rejection, jump_ratio, window, passthrough, recovery | 19 |
| `test_aruco` | YAML schema, lookup, duplicates, malformed files, reload | 34 |

All 130 pass as delivered.
