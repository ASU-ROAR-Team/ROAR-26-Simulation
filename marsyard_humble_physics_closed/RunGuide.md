# Mars Yard Simulation - Quick Run Guide

This guide explains how to clean up previous runs, launch the Mars Yard environment, spawn the integrated rover, and control it.

---

## 0. Clean Up Previous Runs
Before starting, ensure all old simulation processes are terminated. Run this command in any terminal:
```bash
pkill -f -9 "ign|gazebo|ros2|parameter_bridge" || true
```
*(Note: You can safely ignore any "Operation not permitted" warnings for system background services).*

---

## 1. Quick Start Commands

### Step A: Build the Workspace
Ensure the workspace is built and sourced:
```bash
# Navigate to the workspace root
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/marsyard_humble_physics_closed(1)/marsyard_humble_physics_closed

# Build and source
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step B: Launch the Mars Yard World
Start the empty Mars Yard environment. This launches Ignition Gazebo Fortress and bridges the simulation `/clock` to ROS 2:
```bash
ros2 launch marsyard marsyard.launch.py
```

### Step C: Play/Unpause the Simulation
Before spawning the robot, you **must** unpause the simulation so the clock starts ticking.
*   **Via GUI**: Click the **Play** button in the bottom-left corner of the Ignition Gazebo window.
*   **Via CLI** (in a new terminal):
    ```bash
    ign service -s /world/marsyard/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'
    ```

### Step D: Spawn the Complete Rover + Arm
With the simulation unpaused, run the spawn command in a separate terminal:
```bash
# Source the workspace in the new terminal first
source /opt/ros/humble/setup.bash
source install/setup.bash

# Spawn the complete rover (rover + arm)
ros2 launch marsyard spawn_robot.launch.py \
  robot_name:=roar_rover \
  urdf_path:=$(ros2 pkg prefix roar_simulation)/share/roar_simulation/urdf/roar_complete_sim.urdf.xacro \
  x:=0.0 y:=0.0 z:=4.0 yaw:=0.0
```
*(Note: Spawning the robot while the simulation is playing allows the joint state broadcaster and differential drive controller to activate automatically without timing out).*

#### Manual Controller Activation & Troubleshooting (If Spawner Timed Out)
If the simulation was paused during spawning, the controllers will load as `inactive`. Once you unpause the simulation, you can check the status and activate them manually:

1. **Check controller states:**
   ```bash
   ros2 control list_controllers
   ```
2. **Activate the controllers if they are inactive:**
   ```bash
   ros2 control set_controller_state diff_drive_controller active
   ros2 control set_controller_state joint_state_broadcaster active
   ```

### Step E: Control the Robot
You can control the rover using either the GUI Teleop panel or the keyboard.

#### Option 1: GUI Teleop Panel (Recommended)
This launches an interactive control window with speed sliders, buttons, a drag-and-steer visual touchpad, and W/A/S/D keyboard bindings:
```bash
# In a new terminal
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/marsyard_humble_physics_closed(1)/marsyard_humble_physics_closed
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 gui_teleop.py
```

#### Option 2: Keyboard Teleoperation
Run the standard keyboard teleop node:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Use the keyboard keys (e.g., `u`, `i`, `o`, `j`, `k`, `l` or arrow keys depending on configuration) to drive the robot.

---

## 2. Using the Map for Perception & Scans

The Mars Yard environment is fully prepared for perception-based mapping:

* **Visual Mesh (`mars_yard_only_textured.obj`)**: Used by camera and depth sensors. It is fully textured (`marsyard_orthophoto_alpha.png`) and features physical slopes, craters, and obstacles.
* **Collision Mesh (`mars_yard_collision_lowpoly.obj`)**: The physical shape of the terrain that the wheels roll on, optimized to run physics calculations dynamically without simulation lag.
* **Point Clouds & Heightmaps**: 
  If your robot's URDF includes 3D LiDAR, 2D LiDAR, or depth camera sensor plugins, they will automatically scan the environment's meshes. You can feed the resulting sensor topics (such as `/scan` or `/camera/depth/points`) into your perception module or costmap generator to dynamically scan the terrain and generate heightmaps or 2D occupancy costmaps.

---

## 3. Manually Spawning Obstacles & Rocks

To help you plan obstacle placements, you can spawn individual rocks dynamically at any coordinate while the simulator is running.

### Spawning a Rock
Run the helper script from the workspace root:
```bash
./spawn_rock.sh <name> <x> <y> <z> <size> [static: true/false]
```
For example, to spawn a static rock named `rock_1` at `x=2.5`, `y=-1.5`, `z=1.0` with a radius of `0.4` meters:
```bash
./spawn_rock.sh rock_1 2.5 -1.5 1.0 0.4 true
```

*   **Collidable vs. Ghost Rocks:**
    *   **Collidable:** Keeps the `<collision>` tag in the model description. The physics engine will calculate contact forces (the helper script does this by default).
    *   **Non-Collidable (Ghost):** If you omit the `<collision>` tag in the model and only keep the `<visual>` tag, the rock is visible, but the robot will drive straight through it without interacting.
*   **Static vs. Dynamic:**
    *   **Static (`true`):** The rock is anchored in the air/ground. It cannot fall due to gravity and cannot be pushed.
    *   **Dynamic (`false`):** The rock obeys gravity, will fall to the terrain surface, and can be pushed/rolled if hit by the rover.
