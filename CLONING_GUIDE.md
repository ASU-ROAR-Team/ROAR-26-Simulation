# 🚀 ASU ROAR 2026 — Team Onboarding & Cloning Guide

[![Branch](https://img.shields.io/badge/Branch-end__point-E10600?logo=git&logoColor=white)](https://github.com/ASU-ROAR-Team/ROAR-26-Simulation/tree/end_point)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-FF6600?logo=gazebo&logoColor=white)](https://gazebosim.org/home)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04_LTS-E95420?logo=ubuntu&logoColor=white)](https://ubuntu.com/)

This guide provides step-by-step instructions for any team member, mentor, or developer to clone, build, and run the **ASU ROAR 2026 European Rover Challenge (ERC)** autonomous system on a fresh machine.

---

## 💻 1. System Requirements

- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill
- **Physics Engine**: Ignition Gazebo Fortress
- **RAM**: Minimum 8 GB (16 GB recommended)
- **Graphics**: Dedicated or integrated GPU supporting OpenGL 3.3+

---

## 📦 2. Pre-Requisite Dependencies Installation

Open a terminal and run the following commands to install required ROS 2 and system libraries:

### Step 2.1 — Install ROS 2 Humble & Gazebo Fortress Packages
```bash
sudo apt update && sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ign-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-nav2-costmap-2d \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-pluginlib \
  ros-humble-rqt-joint-trajectory-controller \
  ros-humble-teleop-twist-keyboard
```

### Step 2.2 — Install Python Dependencies
```bash
sudo apt install -y python3-tk python3-pip
pip3 install numpy opencv-python matplotlib PyQt5
```

---

## 📥 3. Cloning the Repository

Always clone into `~/ROAR_2026` on the **`end_point`** branch:

### Option A — Using HTTPS (Recommended)
```bash
git clone -b end_point https://github.com/ASU-ROAR-Team/ROAR-26-Simulation.git ~/ROAR_2026
```

### Option B — Using SSH (For developers with SSH keys configured)
```bash
git clone -b end_point git@github.com:ASU-ROAR-Team/ROAR-26-Simulation.git ~/ROAR_2026
```

---

## ⚙️ 4. Building the Workspace

Navigate to the workspace and execute the crash-safe sequential build:

```bash
cd ~/ROAR_2026

# Source ROS 2 Humble base
source /opt/ros/humble/setup.bash

# Build all 29 packages with memory throttling
colcon build --symlink-install --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

> **Why sequential mode?**  
> NavStack controllers (like MPPI) utilize heavy SIMD C++ templates (`xtensor`/`xsimd`). Running with `--parallel-workers 1` and `-DBUILD_TESTING=OFF` guarantees that memory usage stays under 1.2 GB, preventing laptop freezing or out-of-memory (OOM) crashes.

---

## 🔍 5. Verifying Your Installation

Once compilation finishes, verify the environment:

### Step 5.1 — Source the Workspace
```bash
source ~/ROAR_2026/install/setup.bash
```

### Step 5.2 — Confirm Packages are Indexed (Should list 29 packages)
```bash
ros2 pkg list | grep -E 'roar|mppi|dstar|referee|panel'
```

### Step 5.3 — Fast Map Preview Smoke Test
Test that assets, heightmaps, and waypoints load cleanly:
```bash
python3 src/navigation/scripts/preview_mission.py --world 1 --wp 0 --save /tmp/test_preview.png
```
*(If `/tmp/test_preview.png` is generated with a file size $> 100\,\text{KB}$, your setup is 100% operational).*

---

## 🎮 6. Running the System

### Method 1: The Mission Control GUI (Easiest)
Launches the full ASU ROAR telemetry dashboard to manage simulations, Nav2, waypoints, and referee reports from a single window:

```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/navigation/scripts/asu_roar_gui.py
```

---

### Method 2: Manual Terminal CLI

#### Terminal 1 — Launch Gazebo Simulation
```bash
cd ~/ROAR_2026/src/roar2026/dev_environment
bash launch_test.sh noise world1
# Options:
#   Rover: clean | noise | full
#   World: world1 | world2 | world3
#   RViz:  add 'rviz' to launch RViz alongside Gazebo
```

#### Terminal 2 — Launch Autonomous Nav2 Stack
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch navstack_nav2 roar_nav2_only.launch.py
```

#### Terminal 3 — Launch Automated Referee Engine
```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch referee_engine referee.launch.py mission:=mission00
```

---

## 🛠️ 7. Troubleshooting & Common Issues

### Issue 1: `Qt platform plugin "xcb"` error
**Fix**: Ensure `python3-tk` and XCB libraries are present:
```bash
sudo apt install -y libxcb-xinerama0 libxcb-cursor0
export QT_QPA_PLATFORM=xcb
```

### Issue 2: Gazebo or ROS 2 nodes won't stop / port locked
**Fix**: Clean up any background simulation processes and shared memory:
```bash
pkill -9 -f "ign gazebo"
pkill -9 -f "ros2"
pkill -9 -f "rviz2"
rm -rf /dev/shm/fastrtps_*
```

### Issue 3: Missing local dependencies for MPPI
**Note**: `xsimd` and `xtensor` are already bundled in `src/navstack/local_deps` and automatically configured by CMake. No manual installation is needed!

