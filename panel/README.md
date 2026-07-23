
# Panel Simulation Component (`src/panel`)

This directory contains 3D meshes, URDF/XACRO descriptions, `ros2_control` hardware interface definitions, and simulation environments for the control panel assembly, modular switches, rotary knobs, and interactive hardware sensors.

It is integrated as a subfolder of the main ROAR simulation workspace (`Simulation_ws`).

---

## 📂 Workspace Structure

This directory is organized into modular description packages under `src/` to define the control panel and its interactive components:

### Core Orchestration Package
* **`erc_panel_sim`**: Main orchestration package. Manages Gazebo simulation launches, loads `panel_world.sdf` physics configurations, and handles complete URDF assemblies.

### Modular Component Descriptions
* **`panel_body_description`**: Primary 3D meshes, frame materials, and main housing XACRO definitions.
* **`switch_description`**: Modular toggle switches and rotary control parameters.
* **`main_switch_description`**: Master toggle control switch URDF, meshes, and hardware interfaces.
* **`rotary_description`**: Rotary dials and potentiometer knobs.
* **`socket_description`**: Power/charging interface sockets (e.g., IEC320 C14).
* **`led_indicator_description`**: Status indicators and light meshes.
* **`electromagnet_description`**: Electromagnet mounting brackets and disc assemblies.
* **`aruco_description`**: Visual ArUco markers (IDs 1, 2, 3) for perception, pose estimation, and robotic alignment.
* **`plate_description`**: Structural mounting plates and panel surfaces.
* **`row_description`**: Multi-switch row alignment profiles.
* **`slab_description`**: Base slab models and supporting ground geometry.

---

## ⚡ Physics & Performance Optimizations

To ensure high Real-Time Factor (RTF) in Gazebo while simulating 21+ active joints, the following optimizations are implemented:

1. **Primitive Collision Models:** All moving joints (switches, knobs, sockets) and structural bodies use simplified collision primitives (`<box>`, `<cylinder>`) derived from exact mesh bounding boxes, keeping physics calculations lightweight.
2. **Tuned Physics Engine (`panel_world.sdf`):**
   * **Physics Timestep:** Set to `4ms` (`0.004s`) for a 4x reduction in physics solver overhead.
   * **Real-time Update Rate:** Scaled to `100 Hz`.
   * **Rendering:** Dynamic shadows disabled to minimize GPU rendering bottlenecks.

---

## 🛠️ Prerequisites & Dependencies

Ensure ROS 2 Humble desktop and required simulation tools are installed:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers

```

---

## 🚀 Building the Workspace

Always build from the root of your workspace (`~/Simulation_ws`):

### Build All Workspace Packages

```bash
cd ~/Simulation_ws
colcon build --symlink-install
source install/setup.bash

```

### Build Only the Panel Packages

To build or test changes strictly within the panel components:

```bash
cd ~/Simulation_ws
colcon build --packages-select erc_panel_sim panel_body_description switch_description electromagnet_description led_indicator_description main_switch_description plate_description rotary_description row_description slab_description socket_description aruco_description
source install/setup.bash

```

---

## 📊 Launching & Telemetry Verification

### 1. Launch the Main Panel Simulation

Launch Gazebo Sim with the optimized panel world and `ros2_control` interfaces:

```bash
source ~/Simulation_ws/install/setup.bash
ros2 launch erc_panel_sim test_component.launch.py

```

### 2. Verify Joint State Telemetry

All 21 movable joints across the panel publish real-time angular positions to ROS 2:

```bash
ros2 topic echo /joint_states

```

### 3. Test Switch/Knob Physical Interaction

* **Gazebo GUI Force:** In Gazebo, select any individual child link in the **Entity Tree** (e.g., `rotary_1_knob_link` or `row_1_switch_link`), open `Plugins -> Apply Forces`, and apply a small torque (e.g., `1.0 N·m`).
* **Gravity Test:** Drop a 3D shape (e.g., Sphere) from the Gazebo toolbar onto a switch to verify collision response and physical state changes in `/joint_states`.

```

---

## 📤 Commands to Update & Push to Git

To write this file, stage it, commit, and push directly to your branch:

```bash
# 1. Update panel/README.md with the content above
cat << 'EOF' > panel/README.md
# Panel Simulation Component (`src/panel`)

This directory contains 3D meshes, URDF/XACRO descriptions, `ros2_control` hardware interface definitions, and simulation environments for the control panel assembly, modular switches, rotary knobs, and interactive hardware sensors.

It is integrated as a subfolder of the main ROAR simulation workspace (`Simulation_ws`).

---

## 📂 Workspace Structure

This directory is organized into modular description packages under `src/` to define the control panel and its interactive components:

### Core Orchestration Package
* **`erc_panel_sim`**: Main orchestration package. Manages Gazebo simulation launches, loads `panel_world.sdf` physics configurations, and handles complete URDF assemblies.

### Modular Component Descriptions
* **`panel_body_description`**: Primary 3D meshes, frame materials, and main housing XACRO definitions.
* **`switch_description`**: Modular toggle switches and rotary control parameters.
* **`main_switch_description`**: Master toggle control switch URDF, meshes, and hardware interfaces.
* **`rotary_description`**: Rotary dials and potentiometer knobs.
* **`socket_description`**: Power/charging interface sockets (e.g., IEC320 C14).
* **`led_indicator_description`**: Status indicators and light meshes.
* **`electromagnet_description`**: Electromagnet mounting brackets and disc assemblies.
* **`aruco_description`**: Visual ArUco markers (IDs 1, 2, 3) for perception, pose estimation, and robotic alignment.
* **`plate_description`**: Structural mounting plates and panel surfaces.
* **`row_description`**: Multi-switch row alignment profiles.
* **`slab_description`**: Base slab models and supporting ground geometry.

---

## ⚡ Physics & Performance Optimizations

To ensure high Real-Time Factor (RTF) in Gazebo while simulating 21+ active joints, the following optimizations are implemented:

1. **Primitive Collision Models:** All moving joints (switches, knobs, sockets) and structural bodies use simplified collision primitives (`<box>`, `<cylinder>`) derived from exact mesh bounding boxes, keeping physics calculations lightweight.
2. **Tuned Physics Engine (`panel_world.sdf`):**
   * **Physics Timestep:** Set to `4ms` (`0.004s`) for a 4x reduction in physics solver overhead.
   * **Real-time Update Rate:** Scaled to `100 Hz`.
   * **Rendering:** Dynamic shadows disabled to minimize GPU rendering bottlenecks.

---

## 🛠️ Prerequisites & Dependencies

Ensure ROS 2 Humble desktop and required simulation tools are installed:

```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers

```

---

## 🚀 Building the Workspace

Always build from the root of your workspace (`~/Simulation_ws`):

### Build All Workspace Packages

```bash
cd ~/Simulation_ws
colcon build --symlink-install
source install/setup.bash

```

### Build Only the Panel Packages

To build or test changes strictly within the panel components:

```bash
cd ~/Simulation_ws
colcon build --packages-select erc_panel_sim panel_body_description switch_description electromagnet_description led_indicator_description main_switch_description plate_description rotary_description row_description slab_description socket_description aruco_description
source install/setup.bash

```

---

## 📊 Launching & Telemetry Verification

### 1. Launch the Main Panel Simulation

Launch Gazebo Sim with the optimized panel world and `ros2_control` interfaces:

```bash
source ~/Simulation_ws/install/setup.bash
ros2 launch erc_panel_sim test_component.launch.py

```

### 2. Verify Joint State Telemetry

All 21 movable joints across the panel publish real-time angular positions to ROS 2:

```bash
ros2 topic echo /joint_states

```

### 3. Test Switch/Knob Physical Interaction

* **Gazebo GUI Force:** In Gazebo, select any individual child link in the **Entity Tree** (e.g., `rotary_1_knob_link` or `row_1_switch_link`), open `Plugins -> Apply Forces`, and apply a small torque (e.g., `1.0 N·m`).
* **Gravity Test:** Drop a 3D shape (e.g., Sphere) from the Gazebo toolbar onto a switch to verify collision response and physical state changes in `/joint_states`.


