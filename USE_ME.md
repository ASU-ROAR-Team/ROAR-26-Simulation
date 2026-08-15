# Quick Start Guide: Development Environment (`dev_environment`)

Welcome to the Development Sandbox! Before you ever launch the automated Navigation GUI or run headless batch tests, this is where you can manually spawn the rover into the simulation, drive it around, and verify that your generated worlds look correct.

## 1. How to Generate New Worlds
Before launching, you may want to generate fresh randomized rock and ArUco layouts.

**To rebuild the worlds from scratch:**
```bash
cd ../navMission_setup
bash build_full_worlds.sh
```
This script will construct `world1.world` (8 rocks), `world2.world` (50 rocks), and `world3.world` (90 rocks) and automatically place them into the `dev_environment/worlds/` folder for you to test.

## 2. Launching the Simulator
The `dev_environment` comes with a custom script (`launch_test.sh`) that safely cleans up old Gazebo instances, sources your workspace, and spawns the rover exactly where you want it.

**Command Syntax:**
```bash
bash launch_test.sh <rover_config> <world_name> [rviz]
```

### Options:
* **`<rover_config>`**: 
  * `clean` - Standard rover configuration.
  * `noise` - Rover with simulated sensor noise.
  * `full` - Full, heavyweight simulation model.
* **`<world_name>`**: Which world to load (`world1`, `world2`, or `world3`).
* **`[rviz]`**: (Optional) Add the word `rviz` at the end to automatically pop open an RViz2 window so you can view camera feeds, lidar scans, and tf trees.

### Examples:
**Example A:** Launching a simple test in World 1 (8 rocks) without RViz:
```bash
bash launch_test.sh clean world1
```

**Example B:** Launching the dense World 3 (90 rocks) with RViz enabled:
```bash
bash launch_test.sh clean world3 rviz
```

## 3. What to do if Gazebo gets stuck?
Sometimes Gazebo or ROS2 nodes refuse to close properly, preventing you from launching a new test. The `launch_test.sh` script is designed to automatically run a deep cleanup (killing all stale `ruby`, `ign gazebo`, and `ros2` zombie processes) before it boots up, so you should always use this script rather than launching Gazebo manually.
