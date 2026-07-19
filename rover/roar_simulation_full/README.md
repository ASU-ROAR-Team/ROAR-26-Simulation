# 🚀 roar_simulation_full

This package contains the **Full** simulation configuration for the ROAR Rover.

*   **Full**: [Recommended] Includes active environmental degradation, encoder pulse simulation, and the 6-DOF robotic arm.

---

## 🛠️ Compilation

Build this package from the workspace root (`simulation_ws/`):

```bash
cd ~/Desktop/ROAR/simulation_ws/
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Launching the Simulation

We recommend using the clean launch script `launch_sim.sh` provided in this directory. It automatically kills stale Gazebo/ROS2 processes and cleans up FastDDS shared memory.

```bash
cd src/rover/roar_simulation_full
bash launch_sim.sh rviz
```

Alternatively, you can launch it using standard ROS 2 command lines:

```bash
ros2 launch roar_simulation_full basic_rover.launch.py start_rviz:=true
```

---

## 🎮 Driving the Rover (Teleop)

Run the Tkinter-based teleoperation GUI:

```bash
ros2 run roar_simulation_full teleop.py
```
*(Make sure to click on the GUI window to focus it, then use the arrow keys to drive).*

---

## 📖 Detailed Documentation

For a full description of the simulation architecture, sensor topics, control systems, and configuration parameters, please refer to the main **[Rover Simulation README](../README.md)**.
