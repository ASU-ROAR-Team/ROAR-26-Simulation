# ROAR Mars Yard Simulation - Continuation Guide

This file provides a quick reference to the project status, structural changes, and runtime commands to help you continue working and testing the simulation.

---

## 1. Current Workspace Structure

All key workspaces have been decoupled and moved directly to the root of `MARS_YARD_INIT`:
```text
MARS_YARD_INIT/
├── FinalYard/                                  # High-fidelity Final Mars Yard workspace (no symlinks)
│   ├── src/
│   │   └── marsyard/                           # Final Mars Yard simulation package
│   │       ├── launch/
│   │       │   ├── marsyard.launch.py          # Modified to set plugin paths & clock bridge
│   │       │   └── spawn_robot.launch.py        # Ported spawing launch script
│   └── run_marsyard.sh
├── marsyard_humble_physics_closed/             # Closed-Physics Mars Yard workspace
│   ├── src/
│   │   └── marsyard/
│   │       ├── launch/
│   │       │   ├── marsyard.launch.py
│   │       │   └── spawn_robot.launch.py
│   └── run_marsyard.sh
├── ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/ # Rover + Arm package/repo
│   ├── roar_simulation/                        # Package for the Rover + Arm URDF/launch
│   └── src/
├── environment_rocks/                          # Environment Rock assets and generation script
│   ├── rocks_ws/                               # Subdirectories rock_1 through rock_9
│   ├── rock_generator.py                       # Python rock generator script
│   └── README.md                               # Rock generator config and SDF anatomy guide
├── gui_teleop.py                               # Standalone rover-agnostic & world-agnostic GUI
├── README.md                                   # Root master run guide
└── continue.md                                 # This file
```

---

## 2. Key Code Enhancements

1. **Self-Contained FinalYard Launchers**:
   - `FinalYard/src/marsyard/launch/marsyard.launch.py` now includes the `/clock` parameter bridge and appends `roar_simulation` to the Gazebo resource path dynamically, alongside `/opt/ros/humble/lib` for the controllers.
2. **Agnostic Teleop GUI**:
   - `gui_teleop.py` is at the root folder. You can dynamically set/update the cmd_vel topic name inside the GUI or launch it from the CLI with `--topic=<custom_topic>`.
3. **Rock Generator (`rock_generator.py`)**:
   - Dynamically spawns visual gltf (`.glb`) and collision (`.stl`) rocks using a **Settle-and-Freeze** physics process (spawned dynamically to fall to the terrain, then frozen in place as static models). Ghost rocks have their `<collision>` tags removed upon settling.

---

## 3. Step-by-Step Command Flow to Resume Testing

### Step 0: Clean Up Background Processes
```bash
pkill -f -9 "ign|gazebo|ros2|parameter_bridge" || true
```

### Step 1: Build the Workspaces
Compile the workspaces in order (Rover model -> World package):
```bash
# 1. Build Rover Workspace
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. Build and Launch Selected World (e.g. FinalYard)
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard
source /opt/ros/humble/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch marsyard marsyard.launch.py
```

### Step 2: Unpause the Gazebo Clock
Click **Play** in the bottom-left corner of the Gazebo window, or run this in a new terminal:
```bash
ign service -s /world/marsyard/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'
```

### Step 3: Spawn the Rover
In a new terminal:
```bash
source /opt/ros/humble/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard/install/setup.bash
ros2 launch marsyard spawn_robot.launch.py robot_name:=roar_rover
```

### Step 4: Run the Rock Generator
In a new terminal:
```bash
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/environment_rocks
source /opt/ros/humble/setup.bash
python3 rock_generator.py
```

### Step 5: Control the Rover
In a new terminal:
```bash
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT
source /opt/ros/humble/setup.bash
python3 gui_teleop.py
```
*(You can verify collisions by driving the rover into the spawned rocks).*
