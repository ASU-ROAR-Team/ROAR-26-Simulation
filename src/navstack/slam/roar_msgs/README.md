# roar_msgs

Custom ROS2 message definitions for the ASU ROAR Team autonomous navigation
stack (ERC 2026).

## Messages

### `ArucoDetection.msg`

A single ArUco marker detection, published by the perception module
(or by `aruco_simulator_node` in simulation).

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp + camera frame id |
| `id` | `int32` | ArUco marker ID (0–9 in sim; 51–64 at ERC per rules §7.3.2) |
| `pose_in_camera` | `geometry_msgs/Pose` | Marker pose in the **camera optical frame** (x=right, y=down, z=forward). Only the position is consumed by the localization filter — orientation is informational. |
| `distance` | `float32` | Euclidean camera→marker distance [m]. Used for range filtering and distance-scaled observation noise. |

### `ArucoDetectionArray.msg`

All marker detections from one camera frame.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Frame timestamp |
| `detections` | `ArucoDetection[]` | Zero or more detections |

An **empty array is still published** every frame — downstream nodes can use
the timestamps to know perception is alive.

## The contract with perception

This package is the interface agreement between the perception team and the
localization module:

1. Perception detects a marker → fills `id` + `pose_in_camera` + `distance`.
2. Localization looks up the marker's **known world position** (from the
   competition map, `aruco_map.yaml`) using `id`.
3. Localization computes the implied rover position and applies it as a
   high-confidence absolute correction.

Markers are **passive landmarks** — the rover never needs to drive to them.

## Build

No dependencies beyond `std_msgs` / `geometry_msgs`. Must be built **before**
`roar_localization` and `roar_slam_testing`:

```bash
colcon build --packages-select roar_msgs
source install/setup.bash
```
