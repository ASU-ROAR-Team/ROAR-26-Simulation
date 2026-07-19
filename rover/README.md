<div align="center">

# 🚀 ROAR 26 Simulation — Rover & 6-DOF Arm

[![ROS2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

*A comprehensive ROS 2 simulation of the ROAR rover equipped with a 6-DOF robotic arm, operating within the Marsyard 2024 environment. Built for the ASU ROAR Team's ERC 2026 preparation.*

[Features](#-key-features) • [Architecture](#-system-architecture--data-flow) • [Getting Started](#-launching-procedures) • [Controls](#-controlling-the-simulation) • [Degradation Model](#-environmental-degradation-noise-nodes)

</div>

---

## ✨ Key Features
- **Accurate Physics**: Uses Ignition Gazebo (Fortress) for high-fidelity rigid body and wheel traction dynamics.
- **Hardware Parity**: Direct shared-memory access via `ign_ros2_control` mimicking our real `diff_drive_controller` and `joint_trajectory_controller`.
- **Active Degradation Simulation**: Stochastic Martian dust injection, quadratic depth camera decay, and ground-truth-linked dynamic sun glare.
- **Hardware-Matched Encoders**: Real-time simulated encoder pulse conversion matching the IESKF module's WIO specification.

---

## 🏗️ System Architecture & Data Flow

This simulation establishes a robust bridge between the **Gazebo Fortress** physics engine and **ROS 2 Humble**.

### 1. The ROS 2 ↔ Gazebo Bridge
Because Gazebo utilizes its own transport protocol (`ignition.msgs`), the `ros_gz_bridge` seamlessly handles message translation:
- **Time Sync:** Gazebo provides `/clock` to ensure ROS 2 nodes operate in perfect sync with physics.
- **Sensors:** Raw camera matrices and IMU physics are translated into standard `sensor_msgs`.
- **Transforms:** Ground-truth pose is converted into `tf2_msgs/msg/TFMessage` for precise internal tracking.

### 2. ROS 2 Control (Hardware Interfaces)
The URDF utilizes the `ign_ros2_control` plugin. This bypasses topic-level bridging for mechanical joints, providing high-frequency shared-memory access for the `controller_manager`.

### 3. Core ROS 2 Topics

#### 🚗 Rover Control & State
| Topic | Type | Description |
|-------|------|-------------|
| 🟢 `/cmd_vel` | `geometry_msgs/Twist` | Target velocity commands for the differential drive base. |
| 🟢 `/diff_drive_controller/odom` | `nav_msgs/Odometry` | Odometry estimation from simulated wheel encoders. |
| 🟢 `/joint_states` | `sensor_msgs/JointState` | Real-time radians/sec of all mechanical links. |
| 🟢 `/joint_states_updated` | `sensor_msgs/JointState` | Simulated hardware encoder pulses (~201,554 pulses/rev). |

#### 🦾 Arm Control
| Topic | Type | Description |
|-------|------|-------------|
| 🟢 `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Spline waypoints to move the 6-DOF arm. |
| 🟢 `/arm_controller/state` | `control_msgs/...State` | Closed-loop feedback indicating current arm execution error. |

#### 👁️ Perception & Sensors
| Topic | Type | Description |
|-------|------|-------------|
| 📷 `/zed2i/image_raw` | `sensor_msgs/Image` | Pure, unaffected RGB output direct from Gazebo. |
| 🌫️ `/zed2i/image_raw_updated` | `sensor_msgs/Image` | Degraded RGB simulating Martian dust and glare. |
| 📷 `/zed2i/depth` | `sensor_msgs/Image` | Pure, unaffected Depth map. |
| 🌫️ `/zed2i/depth_updated` | `sensor_msgs/Image` | Degraded Depth map with $0.003 \times Z^2$ decay and blinding. |
| 🧭 `/bno055/data` | `sensor_msgs/Imu` | Acceleration, angular velocity, and orientation data. |
| 🧲 `/bno055/mag` | `sensor_msgs/MagneticField` | 3-axis magnetometer readings. |

---

## 📦 Dependencies & Installation

Before building the workspace, ensure you have the following system and Python dependencies installed on your **Ubuntu 22.04** machine running **ROS 2 Humble**:

### 1. ROS 2 Packages
```bash
sudo apt update
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
                 ros-humble-ign-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-robot-state-publisher ros-humble-xacro \
                 ros-humble-rqt-joint-trajectory-controller
```

### 2. Python Packages
```bash
sudo apt install python3-tk
pip3 install numpy opencv-python
```

---

## 🚀 Launching Procedures

We highly recommend using the provided `roar_sim.sh` script to launch the simulation. This script automatically cleans up stale FastDDS memory, prevents GPU rendering crashes by resetting `ign` processes, and injects the correct library paths for Gazebo.

Ensure your workspace is compiled before running the script:
```bash
cd ~/Desktop/ROAR/simulation_ws/
colcon build --symlink-install
```

We provide three distinct simulation packages depending on your testing requirements:

### 1. "Clean" Simulation (`roar_rover_clean`)
Bypasses the degradation nodes and arm controllers. Best for testing pure navigation algorithms, path planning, and kinematics without environmental interference.
```bash
cd src/rover/roar_rover_clean
bash launch_sim.sh rviz
```

### 2. "Noise" Simulation (`roar_rover_noise`)
Includes active environmental degradation (sun glare) and encoder pulse simulation. Perfect for testing the final perception and state estimation stack.
```bash
cd src/rover/roar_rover_noise
bash launch_sim.sh rviz
```

### 3. "Full" Simulation (`roar_simulation_full`)
**[Recommended]** Includes active environmental degradation, encoder pulse simulation, AND the 6-DOF robotic arm.
```bash
cd src/rover/roar_simulation_full
bash launch_sim.sh rviz
```

> **Note on the Boot Sequence:**
> To prevent `ros2_control` race conditions, the launch files execute a carefully timed sequence: 
> Spawns the rover -> Waits 12s -> Loads Broadcasters -> Waits 1s -> Loads Controllers -> Waits 10-20s -> Launches RViz.

---

## 🎮 Controlling the Simulation

### Driving the Rover (Teleop)
You can command the differential drive by running the custom teleop script for whichever package you launched:
```bash
ros2 run roar_rover_clean teleop.py
# OR
ros2 run roar_rover_noise teleop.py
# OR
ros2 run roar_simulation_full teleop.py
```
> **Usage:** This will open a small Tkinter GUI window. **You must click the GUI window to focus it.** Once focused, use your **Arrow Keys** to drive the rover. It will automatically stop when you release the keys.

The 6-DOF arm requires properly timed trajectory points. The expected methodology is to use **MoveIt2** to calculate Inverse Kinematics (IK) and safely path-plan collision-free trajectories. 

### Manual GUI Control (Slider Bars)
If you want to manually control each joint using a graphical interface with slider bars (the "node with the bar"), you can use the built-in ROS 2 trajectory GUI:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```
**How to use:**
1. In the drop-down menu at the top, select `/controller_manager`.
2. Select `arm_controller`.
3. Use the slider bars to move each individual joint in real-time!

### Command Line Control
You can also manually test the arm joints by publishing a `JointTrajectory` message directly to the `/arm_controller/joint_trajectory` topic.

**Example Command (Moves the arm up and opens the gripper):**
```bash
ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [
    'arm_joint_0', 'arm_joint_1', 'arm_joint_2', 
    'arm_joint_3', 'arm_joint_4', 'arm_joint_5',
    'arm_left_gripper', 'arm_right_gripper'
  ],
  points: [
    {
      positions: [0.0, -0.5, 0.5, 0.0, 0.0, 0.0, 0.02, 0.02],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}" -1
```

---

## 🌪️ Environmental Degradation (Noise Nodes)

When running the primary `basic_rover.launch.py`, two custom background nodes intercept the clean Gazebo streams to simulate hardware and environmental reality.

### 1. Encoder Pulse Simulation (`encoder_sim_node.py`)
Our physical IESKF module expects raw hardware encoder pulses, not ideal radians. This node subscribes to `/joint_states` and publishes `/joint_states_updated` containing quantized pulse counts (defaulting to `201,554` pulses per revolution).

### 2. Visual Degradation (`zed_degradation_node.py`)
Intercepts the ZED2i camera streams and injects:
1. **Martian Dust**: A stochastic 1.5% mask applying brownish/orange pixels across the lens.
2. **Quadratic Depth Error**: Calculates a standard deviation of $0.003 \times Z^2$ to exponentially decay depth accuracy over distance.
3. **Dynamic Sensor Blinding (Sun Glare)**: Tracks the 3D vector between the camera lens and a `sun_marker` entity using ground-truth TF. If the sun enters the FOV, it washes out the RGB image and converts local Depth pixels to `NaN` (blinding).

#### ☀️ Editing the Sun Position
The dynamic glare relies on the invisible `sun_marker` entity. If you move the visual Gazebo sky sun in your `.sdf` file, you **must also move the `sun_marker`** to match. 

Edit `src/rover/roar_simulation_full/launch/basic_rover.launch.py` (or noise package, around line 102):
```python
spawn_sun = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-file', sun_marker_file,
               '-name', 'sun_marker', 
               '-x', 'NEW_X', '-y', 'NEW_Y', '-z', 'NEW_Z'], # Update coordinates here
    ...
)
```

---

## ⚙️ Configuration Files Directory

All core tuning parameters reside in `src/rover/roar_simulation_full/configs/` (or noise/clean packages):

| File | Purpose |
|------|---------|
| `diff_controller.yaml` | Wheel separation distance, wheel radius, and base velocity limits. |
| `arm_controller.yaml` | Trajectory execution tolerances and joint PID mapping for the 6-DOF arm. |
| `combined_controllers.yaml` | Top-level `controller_manager` configuration detailing hardware update rates. |
| `joint_names_rover.yaml` | Defines the specific rocker-bogie and wheel joint names from the URDF. |
| `joint_names_arm.yaml` | Defines `joint_1` through `joint_6` and the end-effector joints. |

---

<div align="center">
  <sub>Built with ❤️ by the ASU ROAR Software Team</sub>
</div>
