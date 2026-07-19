# Panel Simulation Component (`src/panel`)

This directory contains the 3D meshes, URDF/XACRO descriptions, and simulation setups for the control panel assembly and its associated controls, modular switches, and sensors. 

It is integrated as a subfolder of the main ROAR simulation workspace (`simulation_ws`).

---

## 📂 Workspace Structure

This directory is organized into modular description packages under `src/` to define the control panel and its interactive components:

### Core Orchestration Package
*   **`erc_panel_sim`**: The main simulation package responsible for orchestration, launching the Gazebo environment, loading the full URDF assemblies, and managing Gazebo simulation configs.

### Modular Component Descriptions
*   **`panel_body_description`**: Contains the primary 3D meshes, materials, and XACRO files for the main panel housing/frame.
*   **`switch_description`**: Defines modular rotary controls and toggle switches mounted on the panel.
*   **`main_switch_description`**: Contains the URDF, meshes, and settings for the main/master toggle control switch.
*   **`rotary_description`**: Additional rotary dials or knobs.
*   **`socket_description`**: Models the sockets (e.g. IEC320 C14 sockets) for charging/power inputs.
*   **`led_indicator_description`**: Standard descriptions and meshes for LEDs and indicator lights.
*   **`electromagnet_description`**: Handles electromagnet hardware models and links.
*   **`aruco_description`**: Holds visual ArUco markers (IDs 1, 2, 3) and XACRO wrappers for perception and alignment calibration.
*   **`plate_description`**: Structural mounting plates.
*   **`row_description`**: Row-specific alignment profiles.
*   **`slab_description`**: Base slab models and physics properties.

---

## 🛠️ Prerequisites & Dependencies

Before building, ensure you have the core ROS 2 Humble packages and joint state tools installed:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-joint-state-publisher-gui ros-humble-xacro
```

---

## 🚀 Building the Packages

Since these packages are located inside `simulation_ws/src/panel/src`, they are automatically detected by `colcon`. Always build from the root of the workspace (`simulation_ws/`):

### Build All Workspace Packages (including the Panel)
```bash
cd ~/Desktop/ROAR/simulation_ws
colcon build --symlink-install
source install/setup.bash
```

### Build Only the Panel Packages
To speed up development when editing panel meshes or URDFs:
```bash
cd ~/Desktop/ROAR/simulation_ws
colcon build --packages-select erc_panel_sim panel_body_description switch_description electromagnet_description led_indicator_description main_switch_description plate_description rotary_description row_description slab_description socket_description aruco_description
```

---

## 📊 Launching & Simulation

Always remember to source the workspace environment after building to register the packages.

### Option 1: Launch the Main Panel simulation in Ignition Gazebo & RViz
To run the setup launch file containing the Ignition Gazebo environment, RViz visualization, and the joint state publisher GUI:
```bash
source ~/Desktop/ROAR/simulation_ws/install/setup.bash
ros2 launch erc_panel_sim test_component.launch.py
```

### Option 2: Launch the Root Helper Simulation (Utility Script)
There is a root-level launch shortcut configured in this directory:
```bash
source ~/Desktop/ROAR/simulation_ws/install/setup.bash
ros2 launch panel/launch/sim.launch.py
```