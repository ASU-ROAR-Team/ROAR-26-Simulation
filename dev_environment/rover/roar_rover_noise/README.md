# 🚀 ROAR 26 Simulation — Rover + 6-DOF Arm on Marsyard

> ROS2 Humble simulation of the ROAR rover with a 6-DOF robotic arm running on Gazebo Fortress with the Marsyard 2024 environment.  
> Part of **ASU ROAR Team's ERC 2026** preparation.

---

## 📁 Package Structure

```
roar_rover_noise/
├── launch/
│   └── basic_rover_mars.launch.py      ← Main launch file
├── urdf/
│   ├── base/
│   │   ├── rover.xacro                 ← Rover base + wheels
│   │   └── rover_simulation.urdf.xacro ← Main URDF (includes everything)
│   ├── arm/
│   │   └── sex_dof_arm.xacro           ← 6-DOF robotic arm
│   └── modules/
│       ├── sensors.xacro               ← Depth camera
│       ├── rover_gazebo.xacro          ← Rover Gazebo plugins
│       └── arm_gazebo.xacro            ← Arm Gazebo plugins
├── meshes/
│   ├── arm/                            ← Arm STL files
│   └── ...                             ← Rover STL files
└── config/
    ├── diff_controller.yaml            ← Differential drive controller
    └── combined_controllers.yaml       ← All controllers config
```

---

## 🛠️ Prerequisites

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Fortress

Install required packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-colcon-common-extensions
```

---

## ⚙️ Setup — Two Workspaces Required

> ⚠️ This simulation needs **two separate workspaces**:
> - **Workspace 1** → Marsyard world environment (`erc2025_remote_sim`)
> - **Workspace 2** → ROAR rover simulation (`roar_rover_noise`)
>
> The Marsyard must be built and sourced **first** so the launch file can find it.




├── marsyard_ws/

│   └── src/erc2025_remote_sim/

└── simulation_ws/

    └── src/roar_rover_noise/

Installation (example paths)

----------------------------
---

### Workspace 1 — Marsyard Environment

```bash
# 1. Create the workspace
mkdir -p ~/marsyard_ws/src

# 2. Clone the Marsyard world
cd ~/marsyard_ws/src
git clone https://github.com/EuropeanRoverChallenge/ERC-Remote-Navigation-Sim.git erc2025_remote_sim

# 3. Build it
cd ~/marsyard_ws
colcon build

# 4. Source it
source install/setup.bash
```

---

### Workspace 2 — ROAR Simulation

```bash
# 1. Create the workspace
mkdir -p ~/roar_ws/src

# 2. Clone this repo (arm branch)
cd ~/roar_ws/src
git clone -b Rover_Arm_Marsyard_Simualtion \
  https://github.com/ASU-ROAR-Team/ROAR-26-Simulation.git roar_rover_noise

# 3. Build it
cd ~/roar_ws
colcon build --packages-select roar_rover_noise

# 4. Source it
source install/setup.bash
```

---

## 🚀 Launch

> ⚠️ **Every time you open a new terminal**, you MUST source both workspaces in order.  
> The Marsyard workspace must be sourced **before** the simulation workspace.

```
Terminal Layout:
─────────────────────────────────────────
📂 You can be anywhere in the terminal
   when running these commands.
─────────────────────────────────────────
```

```bash
# Step 1 — Source Marsyard FIRST
source ~/marsyard_ws/install/setup.bash

# Step 2 — Source ROAR simulation SECOND
source ~/roar_ws/install/setup.bash

# Step 3 — Launch
ros2 launch roar_rover_noise basic_rover_mars.launch.py
```

> ⏳ After Gazebo opens, wait **~12 seconds** for the controllers to finish loading.  
> You will see the rover spawn in the Marsyard world with the arm attached.

---

## ✅ Expected Launch Order

```
1. Gazebo Fortress opens with Marsyard 2024 world
2. Rover spawns after ~2 seconds
3. joint_state_broadcaster loads after ~12 seconds
4. diff_drive_controller loads after broadcaster is ready
5. Arm appears on rover ✅
```

---

## 🤖 What's Included

| Component | Description |
|---|---|
| ROAR Rover | Mecanum wheel rover with full URDF |
| 6-DOF Robotic Arm | Mounted on rover with position control |
| Depth Camera | Mounted on rover front |
| Marsyard 2024 | Full ERC world environment |
| Diff Drive Controller | Rover movement control |
| Joint State Broadcaster | Publishes all joint states |

---

## 📡 ROS2 Topics

| Topic | Type | Description |
|---|---|---|
| `/diff_drive_controller/cmd_vel_unstamped` | Twist | Rover velocity command |
| `/diff_drive_controller/odom` | Odometry | Rover odometry |
| `/joint_states` | JointState | All joint positions |
| `/robot_description` | String | Full URDF |

---

## ⚠️ Common Issues

| Problem | Fix |
|---|---|
| Gazebo crashes on startup | Run `export QT_QPA_PLATFORM=xcb` before launching |
| `erc2025_remote_sim` not found | Source `~/marsyard_ws/install/setup.bash` first |
| Controllers not loading | Wait longer — they need ~12s after spawn |
| Arm missing in Gazebo | Rebuild: `colcon build --packages-select roar_rover_noise` |
| `package://roar_rover_noise` not found | Source `~/roar_ws/install/setup.bash` |

---

> Do **NOT** copy `build/` or `install/` folders between machines — always rebuild locally.

---

**ASU ROAR Team — ERC 2026** 🚀
