# 🧪 ROAR Dev Environment — Multi-World Testing & Rover Control Guide

[![Module](https://img.shields.io/badge/Module-roar2026-E10600?logo=gazebo&logoColor=white)](file:///home/draaven/ROAR_2026/src/roar2026)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Control](https://img.shields.io/badge/Control-Base_&_6--DOF_Arm-00D9FF)](file:///home/draaven/ROAR_2026/src/roar2026/dev_environment)

The development environment allows standalone testing of rover physics, sensor degradation, multi-world terrain traversal, and 6-DOF robotic manipulator control.

---

## 📁 Directory Layout

```text
dev_environment/
├── rover/                        # Rover model packages
│   ├── roar_rover_clean/         # [clean] Clean kinematic simulation (no noise, no arm)
│   ├── roar_rover_noise/         # [noise] Realistic noise & sensor degradation simulation
│   └── roar_simulation_full/     # [full]  Full rover with active 6-DOF arm + noise models
├── worlds/                       # Generated competition test scenes
│   ├── world1/                   # SDF scene, heightmap.npz, obstacle_data.npy, costmap.npz
│   ├── world2/
│   └── world3/
└── launch_test.sh                # Main automated simulation test runner
```

---

## 🚀 1. Launching Simulations

Before launching, source your workspace:
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch any rover configuration across Worlds 1, 2, or 3 using `launch_test.sh`:

```bash
cd ~/ROAR_2026/src/roar2026/dev_environment
# Usage: bash launch_test.sh <rover_config> <world_name> [rviz]

# 1. Clean simulation (ideal physics, zero noise):
bash launch_test.sh clean world1

# 2. Noisy simulation (Martian dust, glare, encoder pulses):
bash launch_test.sh noise world1

# 3. Full simulation (6-DOF robotic arm + noisy sensors):
bash launch_test.sh full world1

# Append 'rviz' to launch RViz alongside Gazebo:
bash launch_test.sh full world1 rviz
```

---

## 🚗 2. Controlling the Rover Base (For Each Module)

The differential drive base is controlled identically across all three configurations (`clean`, `noise`, `full`) via the standard `/cmd_vel` velocity interface.

### Method A: Interactive Tkinter Teleop (Recommended)
Each module includes a dedicated Tkinter graphical key-capture window. Run the command matching your launched configuration:

```bash
# For clean simulation:
ros2 run roar_rover_clean teleop.py

# For noise simulation:
ros2 run roar_rover_noise teleop.py

# For full simulation (with arm):
ros2 run roar_simulation_full teleop.py
```
> **How to Drive:** Click the small pop-up Tkinter window to focus it. Use the **Arrow Keys** (Up/Down for linear velocity, Left/Right for steering). The rover automatically stops when you release the keys.

---

### Method B: Standard Keyboard Teleop
Drive directly from your terminal using the standard ROS 2 keyboard teleop tool:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel
```

---

### Method C: Direct Command-Line Publishing (`ros2 topic pub`)
Publish velocity commands directly to `/cmd_vel` (`geometry_msgs/msg/Twist`):

#### 1. Drive Forward ($0.5\,\text{m/s}$):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 2. Rotate in Place ($0.4\,\text{rad/s}$ Counter-Clockwise):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}"
```

#### 3. Curve Drive (Forward + Right Turn):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}"
```

#### 4. Emergency Stop:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" -1
```

---

## 🦾 3. Controlling the 6-DOF Robotic Arm (`full` Module)

The 6-DOF robotic manipulator with parallel gripper is active when running the **`full`** configuration (`roar_simulation_full`).

### Method A: Graphical Slider Bars (`rqt_joint_trajectory_controller`)
To control each arm joint interactively using graphical slider bars:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

**Step-by-Step Instructions:**
1. In the top dropdown menu, select **`/controller_manager`**.
2. In the controller dropdown, select **`arm_controller`**.
3. Toggle the controller power icon to **On** (green).
4. Drag the sliders to position `arm_joint_0` through `arm_joint_5` and the gripper fingers in real time!

---

### Method B: CLI Direct Joint Trajectory Presets (`ros2 topic pub`)
Publish trajectory goals directly to `/arm_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).

#### Preset 1 — Stowed / Home Position:
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_joint_0', 'arm_joint_1', 'arm_joint_2', 
    'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
    'arm_left_gripper', 'arm_right_gripper'
  ],
  points: [
    {
      positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}" -1
```

#### Preset 2 — Reach & Inspect Pose (Arm Elevated, Gripper Open):
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_joint_0', 'arm_joint_1', 'arm_joint_2', 
    'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
    'arm_left_gripper', 'arm_right_gripper'
  ],
  points: [
    {
      positions: [0.0, -0.6, 0.8, 0.0, 0.5, 0.0, 0.03, 0.03],
      time_from_start: {sec: 3, nanosec: 0}
    }
  ]
}" -1
```

#### Preset 3 — Ground Sample Pickup Pose (Arm Reaching Down):
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_joint_0', 'arm_joint_1', 'arm_joint_2', 
    'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
    'arm_left_gripper', 'arm_right_gripper'
  ],
  points: [
    {
      positions: [0.0, -1.1, 1.4, 0.0, 0.8, 0.0, 0.03, 0.03],
      time_from_start: {sec: 4, nanosec: 0}
    }
  ]
}" -1
```

#### Preset 4 — Close Gripper (Grasp Object):
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['arm_left_gripper', 'arm_right_gripper'],
  points: [
    {
      positions: [0.0, 0.0],
      time_from_start: {sec: 1, nanosec: 0}
    }
  ]
}" -1
```

---

## 📡 4. Core ROS 2 Topics & Interfaces

| Component | Topic | Type | Description |
| :--- | :--- | :--- | :--- |
| **Base Velocity** | `/cmd_vel` | `geometry_msgs/Twist` | Target velocity for differential drive |
| **Odometry** | `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Wheel odometry state |
| **Hardware Encoders** | `/joint_states_updated` | `sensor_msgs/JointState` | Simulated optical encoder pulses (~201,554 pulses/rev) |
| **Arm Trajectory** | `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 6-DOF arm trajectory command interface |
| **Arm Feedback** | `/arm_controller/state` | `control_msgs/JointTrajectoryControllerState` | Closed-loop joint execution feedback |
| **RGB Camera** | `/zed2i/image_raw_updated` | `sensor_msgs/Image` | Degraded camera feed (Martian dust & glare) |
| **Depth Camera** | `/zed2i/depth_updated` | `sensor_msgs/Image` | Degraded depth image ($0.003 \cdot Z^2$ decay) |
| **IMU** | `/bno055/data` | `sensor_msgs/Imu` | 9-DOF orientation, angular velocity, and acceleration |
| **Ground Truth** | `/world/rover_world/pose/info` | `tf2_msgs/TFMessage` | Absolute simulation ground truth pose |
