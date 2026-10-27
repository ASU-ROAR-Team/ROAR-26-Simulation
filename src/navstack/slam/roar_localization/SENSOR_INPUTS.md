# roar_localization — Sensor Inputs

What `ieskf_node` subscribes to and how each message field is **actually used in the code**
(`src/ieskf_node.cpp`), not what the message docs say. Unused fields are flagged.

The other nodes in this package (`ground_truth_bridge`, `aruco_simulator_node`,
`synthetic_imu_node`) are simulation helpers and don't consume real sensor data.

---

## 1. `sensor_msgs/Imu` — `/imu/data` (~100 Hz, BNO055)

Drives the IESKF **predict** step, ZUPT, and the WO-FIS adaptive noise.

| Field | Used? | What it actually holds / how it's used |
|---|---|---|
| `orientation` | ✅ | Absolute orientation from the BNO055's internal NDOF fusion — the filter **trusts it as ground truth** for attitude (it is never estimated). Rotated by the IMU mount correction (`imu_roll/pitch/yaw_deg`), stored as `current_orientation_`, fed to WIO to project wheel motion into the world frame, and republished directly in `/odom` and TF. |
| `angular_velocity` | ✅ | Body rates in the IMU frame, rotated into body frame. Full vector goes into `filter_->predict()`; the **z component only** is additionally used for the WO-FIS turn-rate input and the ZUPT stationarity check. |
| `linear_acceleration` | ✅ | Specific force in the IMU frame, rotated into body frame. Used in `predict()` and in ZUPT's rolling accel-variance check. |
| `header.stamp` | ⚠️ **UNUSED** | `dt` is computed from the **node's own clock** (`get_clock()->now()`), not the message timestamp. Sensor timestamps are ignored entirely. |
| `orientation_covariance` | ❌ **UNUSED** | The node publishes a hard-coded ~1° orientation variance instead. |
| `angular_velocity_covariance` | ❌ **UNUSED** | Gyro noise comes from the `sigma_ww` parameter. |
| `linear_acceleration_covariance` | ❌ **UNUSED** | Accel noise comes from `sigma_an`, scaled online by the fuzzy `k_Q`. |

## 2. `sensor_msgs/JointState` — `/joint_states`

Wheel encoders. Callback only caches values; they're consumed on the next IMU tick by WIO.

| Field | Used? | What it actually holds / how it's used |
|---|---|---|
| `name` | ✅ | Matched against the `joint_left` / `joint_right` parameters to pick the two wheel joints; all other joints in the message are ignored. |
| `position` | ✅ | **Cumulative wheel rotation; unit is defined by the `pulses_per_rev` parameter, not assumed.** WIO converts position deltas via `metres_per_pulse = 2π·r / pulses_per_rev`. Both launch files set `pulses_per_rev = 2π`, i.e. `position` is treated as cumulative **radians** (ROS-standard, what Gazebo publishes). The code default (2000) instead assumes raw encoder counts — if a real driver publishes counts, set it to PPR × quadrature factor. |
| `velocity` | ❌ **UNUSED** | Wheel speeds are re-derived from position deltas and the IMU-callback `dt` instead. |
| `effort` | ❌ **UNUSED** | — |
| `header.stamp` | ❌ **UNUSED** | Encoder deltas are timed by the IMU callback's clock, not this stamp. |

## 3. `geometry_msgs/PoseWithCovarianceStamped` — `/zed2i/zed_node/pose_with_covariance` (~30 Hz)

ZED2i visual odometry pose → WVO gating → IESKF **position update**.

| Field | Used? | What it actually holds / how it's used |
|---|---|---|
| `pose.pose.position` | ✅ | Camera position in the ZED world frame. Shifted to `base_link` using `current_orientation_` and the `cam_to_base_*` lever arm, then gated by WVO (jump check against accumulated wheel odometry) before being fused. |
| `pose.covariance[0]`, `[7]`, `[14]` | ✅ | Only the three **position diagonal** entries. Their trace is squashed into a 0–100 "ZED confidence" score (`exp(-trace/zed_cov_scale)`) that feeds the VO-FIS to scale measurement noise `k_V`. Not used as an actual covariance in the filter. |
| `pose.pose.orientation` | ❌ **UNUSED** | Orientation is owned by the BNO055; ZED attitude is ignored. |
| `pose.covariance` (other 33 entries) | ❌ **UNUSED** | Orientation block and all off-diagonals are ignored. |
| `header.stamp` | ❌ **UNUSED** | No time alignment with the filter state is done. |

## 4. `roar_msgs/ArucoDetectionArray` — `/aruco/detections` (param `aruco_topic`)

Landmark corrections against the YAML marker map (`aruco_map_path`).

| Field | Used? | What it actually holds / how it's used |
|---|---|---|
| `detections[].id` | ✅ | Looked up in the marker map; detections with unknown IDs are dropped with a throttled warning. |
| `detections[].pose_in_camera.position` | ✅ | Marker position in the ZED **optical** frame (x=right, y=down, z=forward). Transformed via `R_cam_to_body_` + lever arm inside `filter_->updateAruco()`. |
| `detections[].distance` | ✅ | Camera→marker range in metres. Used twice: rejection gate (`aruco_max_distance`, default 15 m) and linear scaling of the measurement noise (`sigma_aruco_base + sigma_aruco_per_m · distance`). |
| `detections[].pose_in_camera.orientation` | ❌ **UNUSED** | Informational only — marker orientation is never observed (BNO055 owns attitude). |
| `header` (array) and `detections[].header` | ❌ **UNUSED** | Neither timestamp nor frame_id is read. |

---

## Summary of unused data

- **No sensor timestamp is used anywhere** — all timing comes from the node's wall/ROS clock at callback time. This is the biggest gap: queued or delayed messages get integrated with the wrong `dt`.
- All incoming covariance fields are ignored except the ZED position diagonal (and even that is only turned into a scalar confidence score) — filter noise is entirely parameter-driven with fuzzy scaling.
- `JointState.velocity` / `effort`, ZED orientation, and ArUco marker orientation are dead weight on the wire.
