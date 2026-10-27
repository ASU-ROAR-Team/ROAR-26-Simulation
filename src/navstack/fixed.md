# Fixed — Changes Made to Get the Nav Stack Working


## 1. Bugs found and fixed 


- **Stale `build/`/`install/` artifacts** for `dstar_navigation`,
  `active_mapping_msgs`, `roar_msgs`, `terrain_mapping` — leftover
  `ament_cmake_python` symlink targets from an older build layout that blocked
  `colcon build` entirely (`failed to create symbolic link ... Is a directory`).
  Cleared just those four packages' `build/`/`install/` folders and rebuilt.
- **Wrong ground-truth `ros_gz_bridge` topic** — `navstack_nav2/launch/
  roar_nav2_only.launch.py`'s `ground_truth_bridge` node was subscribed to
  `/world/marsyard/pose/info`. The world actually launched in this setup
  (`ROAR-26-Simulation/dev_environment/worlds/world1|2|3`) all declare SDF
  world name `rover_world`, so `ros_gz_bridge` was publishing ground truth on
  `/world/rover_world/pose/info` instead — a topic with no subscriber, and
  `ground_truth_bridge` was subscribed to a topic with no publisher. Neither
  side was actually broken; the names just didn't match. Result:
  `ground_truth_bridge` never received anything, never published
  `/zed2i/zed_node/pose_with_covariance`, and `slam_evaluator` hung forever on
  "Waiting for /odom + ground truth ...". Fixed `GT_TOPIC_IN` to
  `/world/rover_world/pose/info`. (Note: `marsyard` is correct for a
  *different* world — navStack's own bundled `slam/worlds/` files — so this
  would need to flip back if you ever launch against that world instead.)



