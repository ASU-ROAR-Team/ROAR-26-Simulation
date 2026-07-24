# 🎮 ROAR Dev Environment: Simulation Guide

This standalone development environment allows you to test the ROAR rover (and its 6-DOF arm) inside generated Marsyard worlds with custom obstacles.

---

## 🚀 1. Launching the Simulation

Use the provided `launch_test.sh` script to start the simulation. 
It requires two arguments: the **rover configuration** and the **world name**.

```bash
cd ~/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/dev_environment

# Format: bash launch_test.sh <clean|noise|full> <world_folder_name> [rviz]

#Clean (no noise, no obstacles)



bash launch_test.sh clean world1 rviz
bash launch_test.sh clean world2 rviz
bash launch_test.sh clean world3 rviz


hwash launch_test.sh noise world1 rviz
bash launch_test.sh noise world2 rviz
bash launch_test.sh noise world3 rviz


bash launch_test.sh full world1 rviz
bash launch_test.sh full world2 rviz
bash launch_test.sh full world3 rviz
```

*Note: The script automatically cleans up old Gazebo processes, builds the workspace if needed, and logs the results to the `results/` folder.*

---

## 🕹️ 2. Controlling the Rover (Teleoperation)

Once the simulation is running, open a **new terminal tab**. 
You need to source the dev environment workspace and then run the teleop node.

```bash
cd ~/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/dev_environment
source install/setup.bash

# Run the teleop script corresponding to the config you launched:
ros2 run roar_rover_clean teleop.py
# OR
ros2 run roar_rover_noise teleop.py
# OR
ros2 run roar_simulation_full teleop.py
```

### How to Drive:
1. A small Tkinter GUI window will pop up.
2. **You must click the GUI window to focus it.**
3. Use the **Arrow Keys** (Up, Down, Left, Right) to drive the rover.
4. Release the keys to stop automatically.

---

## 🦾 3. Controlling the Arm (Full Simulation Only)

If you launched using the `full` configuration (`roar_simulation_full`), the 6-DOF arm controllers are active. You can control the arm using the ROS 2 Joint Trajectory GUI:

Open a **new terminal tab**:
```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

### How to use the Arm GUI:
1. In the drop-down menu at the top, select `/controller_manager`.
2. Select `arm_controller`.
3. Use the slider bars to move each individual joint in real-time!

---

## 🛠️ Recent Fixes & Improvements
* **Spawn Height:** The rover now spawns at `z = 2.5m` (previously `0.5m`). This ensures it safely drops onto the terrain rather than spawning underground when testing on high-elevation areas of the Marsyard or on top of generated rocks.
* **Standalone Models:** The `mars_yard` terrain and `rock_generator` meshes have been embedded directly into `dev_environment/models` to ensure Gazebo can render them without needing external paths.
