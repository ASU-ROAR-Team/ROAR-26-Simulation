# 🚀 roar_rover_noise

This package contains the **Noise** simulation configuration for the ROAR Rover.

*   **Noise**: Includes active environmental degradation (sun glare, camera dust) and encoder pulse simulation. Perfect for testing perception and state estimation.

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
cd src/rover/roar_rover_noise
bash launch_sim.sh rviz
```

Alternatively, you can launch it using standard ROS 2 command lines:

```bash
ros2 launch roar_rover_noise basic_rover.launch.py start_rviz:=true
```

---

## 🎮 Driving the Rover (Teleop)

Run the Tkinter-based teleoperation GUI:

```bash
ros2 run roar_rover_noise teleop.py
```
*(Make sure to click on the GUI window to focus it, then use the arrow keys to drive).*

---

## 📖 Detailed Documentation

For a full description of the simulation architecture, sensor topics, control systems, and environmental degradation parameters, please refer to the main **[Rover Simulation README](../README.md)**.
