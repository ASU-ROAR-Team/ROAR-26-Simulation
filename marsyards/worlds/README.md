# Worlds Package

This is a standalone ROS 2 package designed to organize, store, and launch individual simulation worlds for the ROAR environment.

## Directory Structure

```
simulation_ws/src/marsyards/worlds/
├── CMakeLists.txt              # Build configuration for colcon
├── package.xml                 # Package manifest
├── README.md                   # This document
├── launch/
│   ├── launch_map.launch.py   # Parameterized launch (runs any world)
│   └── marsyard.launch.py     # Shortcut launch (runs marsyard.world)
└── worlds/
    └── marsyard.world         # The simulation world description file
```

---

## Why are `package.xml` and `CMakeLists.txt` required?

In ROS 2, files must be "installed" into the workspace overlay (`simulation_ws/install/`) so that other tools and launching systems can locate them dynamically.
- **`package.xml`**: Informs `colcon build` that this directory is a valid ROS 2 package, names it `worlds`, and lists runtime dependencies (like `ros_gz_sim`).
- **`CMakeLists.txt`**: Instructs the build system to copy the `worlds/` and `launch/` directories into `install/worlds/share/worlds/`. Without this file, running `ros2 launch worlds <launch_file>` will fail because ROS 2 will not find the package or its files in the installation directories.

---

## How to Build and Setup

This package resides inside the `simulation_ws/src/marsyards/` directory.

To build it:
1. Navigate to the `simulation_ws` directory:
   ```bash
   cd ~/Desktop/ROAR/simulation_ws
   ```
2. Build the package using `colcon`:
   ```bash
   colcon build --packages-select worlds
   ```
3. Source the updated workspace:
   ```bash
   source install/setup.bash
   ```

---

## How to Launch Worlds

### Option 1: Parameterized Launch (Any World)
You can launch any world present in the `worlds/` directory using the `world` argument:
```bash
ros2 launch worlds launch_map.launch.py world:=marsyard.world
```

### Option 2: Shortcut Launch (Direct)
You can launch `marsyard.world` directly using the shortcut:
```bash
ros2 launch worlds marsyard.launch.py
```

---

## Parameterized World Generation Integration

This package integrates directly with the **`rock_generator`** package to store and run customized terrain layouts:
- **Dynamic Worlds**: Fused worlds are saved to the `worlds/` folder using the name pattern `w_d{density}_c{collidable_ratio}.world`.
- **Dynamic Launch Files**: Custom launch files are saved to the `launch/` folder using the name pattern `w_d{density}_c{collidable_ratio}.launch.py`.

### How to Launch a Generated Configuration:
Once the rock generator pipeline finishes spawning and fusing obstacles, compile the package:
```bash
colcon build --packages-select worlds
source install/setup.bash
```
Then launch it using its custom launch script:
```bash
ros2 launch worlds w_d{density}_c{collidable_ratio}.launch.py
```

