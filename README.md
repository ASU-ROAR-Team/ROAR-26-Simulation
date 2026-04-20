# ROS 2 Rover Simulation 🚀

Welcome to the ROS 2 Humble Rover Simulation repository.
Please read the following instructions carefully before using the simulation.

---

# Installation Steps

## Step 0: Clone the Repository

Only do this the first time you use the simulation.

```bash
git clone https://github.com/ASU-ROAR-Team/ROAR-26-Simulation roar_onsite_ws
```

This command clones the simulation repository into a workspace directory named `roar_onsite_ws`

---

## Step 0.5: Install Dependencies

This step involve installing dependancies but there is no dependancies ( yet! :) )

---

## Step 1: Build the Workspace

Navigate to the workspace and build it using:

```bash
cd roar_onsite_ws
colcon build --symlink-install
```

After building, remember to source the workspace:

```bash
source install/setup.bash
```

You should source the workspace every time you open a new terminal or rebuild the project.

---

## Step 2: Launch the Simulation

To launch the rover in Gazebo without the robotic arm, use:

```bash
ros2 launch basic_rover.launch.py
```

You can control the simulation by publishing commands to the following topic:

```bash
/diff_drive_controller/cmd_vel_unstamped
```

You can also use the teleoperation keyboard node from another terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

---


## Common Issue: Controller Activation Failure

A common issue, especially when using WSL instead of dual boot, is that the controller may fail to activate.

To solve this:

1. Open the following file:

```bash
src/roar_simulation/launch/basic_rover.launch.py
```

2. Go to the `return LaunchDescription(...)` section near the end of the file.
3. Locate the second node shown in the image.
<img width="981" height="289" alt="Screenshot from 2026-04-21 00-49-28" src="https://github.com/user-attachments/assets/f7a8b6c4-5cf2-4f41-9610-312a48271851" />
4. Increase the startup delay period.
5. Keep increasing the delay until the controller launches successfully.

For WSL users, a delay of around 45 to 60 seconds is recommended.
