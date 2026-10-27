# 🪐 ROAR 2026 — Simulation & Digital Twin Architecture

This module hosts the physics simulation, Gazebo Fortress environments, Marsyard terrain models, sensor degradation pipelines, and the ERC maintenance panel.

---

## 📁 Directory Layout

```text
src/roar2026/
├── coordinate_bridge/        # Metric survey-to-world coordinate transform bridge
├── dev_environment/         # Rover URDF/Xacro models & test world scenes
│   ├── rover/                # roar_rover_clean, roar_rover_noise, roar_simulation_full
│   ├── worlds/               # world1, world2, world3 (SDFs, costmaps, heightmaps)
│   └── launch_test.sh        # Main CLI simulation launch runner
├── marsyards/                # erc_marsyard_2026 terrain model + ArUco 1..15 Gazebo models
├── navMission_setup/         # World & waypoint generation pipeline (build_full_worlds.sh)
└── panel/                    # ERC maintenance panel & 12 joint/switch controllers
```

---

## 🚀 Running Simulations

```bash
cd ${ROAR_WS}
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/roar2026/dev_environment
# Usage: bash launch_test.sh <rover_config> <world_name> [rviz]
bash launch_test.sh clean world1
bash launch_test.sh noise world2 rviz
bash launch_test.sh full world3
```

- **Rover Configurations**:
  - `clean`: Ideal kinematic model, ground truth sensors, zero noise.
  - `noise`: Realistic sensor degradation (Martian dust injection, sun glare, depth camera quadratic decay, noisy IMU & wheel encoder pulses).
  - `full`: Complete rover model with active 6-DOF robotic arm manipulator and noise models.
