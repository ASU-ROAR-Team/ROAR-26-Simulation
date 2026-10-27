# ROAR Perception Testing — Localization-Aware Report

This package produces a live standalone HTML report with:

1. Unique confirmed obstacle detections (no row per frame).
2. Detection-to-ground-truth comparison with position, dimension, IoU and collidability errors.
3. Ground-truth coverage using robot localization and camera FOV. A rock is `MISSED` only after a real visible opportunity; unvisited rocks do not affect recall.
4. Detailed ArUco evaluation.
5. TF tree and localization health checks.
6. A graph after every table.

Default target frame is `map`, base frame is `base_link`, and camera frame is `zed2i_depth_optical_frame`.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch roar_perception_testing perception_benchmark.launch.py \
  ground_truth_path:="$HOME/new_sim_tes/test_sim/ROAR-26-Simulation/dev_environment/worlds/worldData_126/obstacle_data.npy" \
  world_name:=worldData_126 \
  perception_topic:=/detection/lidar_clusters \
  aruco_topic:=/detection/aruco_3d \
  target_frame:=map
```

## Viewpoint-independent GT tracking update

This version transforms each detection into `map`, matches it against an expanded Ground Truth volume, and keeps one logical track per `Rock_ID`.

- Front, side, and rear views may resolve to the same rock.
- Extra detections assigned to the same GT rock are marked `SAME ROCK - DIFFERENT VIEW` and do not count as false positives.
- Unmatched detections include the nearest GT, center distance, GT-volume distance, and a diagnostic reason.
- Matching uses the GT dimensions, adaptive center radius, AABB overlap, and configurable padding.

Main parameters:

```yaml
match_distance_m: 1.0
gt_box_padding_m: 0.45
min_confirmations: 3
noise_min_dimension_m: 0.12
noise_min_volume_m3: 0.004
```
