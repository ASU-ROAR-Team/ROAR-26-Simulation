# Mars Yard Simulation Environment - Updates & Spawning Guide

This document tracks the updates made to the `marsyard` ROS 2 package to support automated bridges and robot spawning.

---

## 1. Summary of Changes

### Workspace Integration
* **`roar_simulation` Integration**
  * Symlinked the `roar_simulation` package from `data/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/roar_simulation` into the workspace `src/` directory.

### Modified Files
* **`package.xml`**
  * Added execution dependencies for `ros_gz_bridge` and `robot_state_publisher` to ensure proper dependency management.
* **`launch/marsyard.launch.py`**
  * Updated to dynamically append `roar_simulation` package share directories to `IGN_GAZEBO_RESOURCE_PATH` and `GZ_SIM_RESOURCE_PATH` so meshes load automatically.
* **`launch/spawn_robot.launch.py`**
  * Added support for dynamically starting `joint_state_broadcaster` and `diff_drive_controller` spawner nodes (with remappings to `/cmd_vel` and `/odom`).
* **`roar_simulation/urdf/roar_complete_sim.urdf.xacro`**
  * Fixed missing `controllers_yaml` parameter when instantiating the `rover_gazebo` macro.
* **`roar_simulation/urdf/modules/arm_gazebo.xacro`**
  * Replaced the deprecated `gazebo_ros2_control/GazeboSystem` hardware plugin with the Ignition-compatible `gz_ros2_control/GazeboSimSystem` plugin.

---

## 2. Launching the Environment (Step 3)

The world simulation now automatically bridges the simulation `/clock` to ROS 2.

### To Launch the World:
```bash
# Source the ROS 2 setup
source /opt/ros/humble/setup.bash

# Build the package
colcon build --symlink-install
source install/setup.bash

# Launch the world (spawns the Mars Yard terrain and starts the clock bridge)
ros2 launch marsyard marsyard.launch.py
```

---

## 3. Spawning a Robot (Step 4)

The template launch file `spawn_robot.launch.py` allows spawning a robot and automatically bridges and remaps control and status topics.

### Parameters supported by `spawn_robot.launch.py`:
| Parameter | Default Value | Description |
| :--- | :--- | :--- |
| `robot_name` | `mars_rover` | The namespace and model name of the robot inside Gazebo. |
| `x` | `0.0` | Spawn coordinate X. |
| `y` | `0.0` | Spawn coordinate Y. |
| `z` | `0.5` | Spawn coordinate Z (above the terrain to avoid mesh collisions). |
| `yaw` | `0.0` | Spawn yaw orientation in radians. |
| `urdf_path` | `""` | (Optional) Path to the robot's URDF/Xacro file. If provided, launches a `robot_state_publisher` for TF transforms. |

### Configured Bridges:
When the robot spawns, the following bridges are launched and remapped:
* **Command Velocity (`/cmd_vel`)**: Bidirectional command velocity between ROS 2 `/cmd_vel` and Gazebo `/model/<robot_name>/cmd_vel`.
* **Odometry (`/odom`)**: Bridges Gazebo `/model/<robot_name>/odometry` to ROS 2 `/odom`.
* **Joint States (`/joint_states`)**: Bridges Gazebo joint updates to ROS 2 `/joint_states`.

### How to use `spawn_robot.launch.py`:

#### Option A: Spawning and Publishing State (Recommended)
If you have a URDF/Xacro file, pass it directly. This will start the `robot_state_publisher` and spawn the entity:
```bash
ros2 launch marsyard spawn_robot.launch.py \
  robot_name:=my_custom_rover \
  urdf_path:=/path/to/my_robot.urdf \
  x:=2.0 y:=5.0 z:=0.5 yaw:=1.57
```

#### Option B: Spawning Only (If Robot State Publisher is run elsewhere)
If your robot description is already published to the `/robot_description` topic by another node:
```bash
ros2 launch marsyard spawn_robot.launch.py \
  robot_name:=my_custom_rover \
  x:=2.0 y:=5.0 z:=0.5 yaw:=1.57
```

---

## 4. Controlling the Rover
Once the world is running and the robot is spawned, you can send command velocities directly using standard ROS 2 tools:

### Teleoperating via CLI:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
This will publish `/cmd_vel` messages, which the bridge will route to the Gazebo physics model automatically.
