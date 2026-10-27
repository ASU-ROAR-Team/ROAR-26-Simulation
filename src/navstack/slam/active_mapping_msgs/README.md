# active_mapping_msgs

Message definitions for the mapping module (mapping team's package,
included verbatim).

## Messages

### `ObstacleDetection.msg`

One detected obstacle as a 3D bounding box.

| Field | Type | Meaning |
|---|---|---|
| `header` | `std_msgs/Header` | Timestamp + **source frame** of `position` |
| `position` | `geometry_msgs/Point` | Center of the bounding box in the source frame |
| `dimensions` | `geometry_msgs/Vector3` | Bbox width (x), length (y), height (z) [m] |
| `confidence` | `float32` | Detection confidence [0, 1] |
| `class_id` | `uint32` | Obstacle class (1 = rock in the test scenario) |

### `ObstacleDetectionArray.msg`

| Field | Type |
|---|---|
| `header` | `std_msgs/Header` |
| `detections` | `ObstacleDetection[]` |

## Frame convention that matters

`active_mapping` transforms each detection from `header.frame_id` into its
map frame via TF — **except** when `header.frame_id` already equals the map
frame, in which case the position is used as-is (no TF lookup).

The simulation test harness (`roar_slam_testing/obstacle_simulator`)
deliberately publishes in the `map` frame to use that short-circuit, because
no `map→odom` TF exists in the sim. The real perception node will publish in
the camera frame, and TF wiring will be designed during real-rover
integration.

## Build

Must be built **before** `active_mapping` and `roar_slam_testing`:

```bash
colcon build --packages-select active_mapping_msgs
source install/setup.bash
```
