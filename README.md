# 🚀 ROAR 26 Simulation — Rover + 6-DOF Arm on Marsyard

> ROS2 Humble simulation of the ROAR rover with a 6-DOF robotic arm running on Gazebo Fortress with the Marsyard 2024 environment.  
> Part of **ASU ROAR Team's ERC 2026** preparation.

---

## 📁 Package Structure

```text
ROAR-26-Simulation/
├── src/
│   ├── roar_simulation/         # Rover configuration and control
│   │   ├── launch/              # Main launch files
│   │   ├── urdf/                # Rover and Arm URDF/Xacro files
│   │   ├── meshes/              # Rover and Arm STL/PNG assets
│   │   └── config/              # Controller YAML configurations
│   └── erc2025_remote_sim/      # Marsyard world environment
│       ├── worlds/              # Marsyard 2024 world file





│       └── models/              # Marsyard and ArUco markers models
🛠️ PrerequisitesUbuntu 22.04ROS2 HumbleGazebo FortressInstall required packages:Bashsudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-controller-manager \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  python3-colcon-common-extensions


⚙️ Setup — Single Workspace⚠️ This simulation is now integrated into a single workspace for easier management and build process.Bash# 1. Create the workspace
mkdir -p ~/roar_ws/src
cd ~/roar_ws/src

# 2. Clone this repository (ensure you are on the correct branch)
git clone -b Rover_Arm_Marsyard_Simualtion \
  [https://github.com/ASU-ROAR-Team/ROAR-26-Simulation.git](https://github.com/ASU-ROAR-Team/ROAR-26-Simulation.git) .

# 3. Build the workspace
cd ~/roar_ws
colcon build

# 4. Source the workspace
source install/setup.bash
🚀 LaunchBash# Every time you open a new terminal:
# Every time you open a new terminal:
cd ~/roar_ws
source install/setup.bash

# Launch the simulation
ros2 launch roar_simulation basic_rover_mars.launch.py


QT_QPA_PLATFORM=xcb before launchingControllers not loadingWait longer — they need ~12s after spawnArm missing in GazeboRebuild: colcon build --packages-select roar_simulationPackage not found errorSource the workspace: source install/setup.bashDo NOT copy build/ or install/ folders between machines — always rebuild locally.ASU ROAR Team — ERC 2026 🚀
