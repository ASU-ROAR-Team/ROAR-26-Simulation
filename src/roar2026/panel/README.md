# 🎛️ ROAR 2026 — Maintenance Panel Sub-Task Digital Twin

[![Module](https://img.shields.io/badge/Module-roar2026-E10600?logo=gazebo&logoColor=white)](file:///home/draaven/ROAR_2026/src/roar2026)
[![Task](https://img.shields.io/badge/Task-ERC_Maintenance_Panel-BB86FC)](file:///home/draaven/ROAR_2026/src/roar2026/panel)

This subsystem models the European Rover Challenge **Maintenance Panel**, featuring interactive toggle switches, continuous rotary knobs, power sockets, status LEDs, and electromagnetic interfaces manipulated by the rover's 6-DOF robotic arm.

---

## 📦 Panel Component Packages (12 ROS 2 Packages)

```text
src/roar2026/panel/
├── panel_body_description/       # Structural frame, mounting brackets & base link
├── plate_description/            # Mounting plate panels
├── slab_description/             # Modular structural partition slabs
├── main_switch_description/      # Heavy high-voltage disconnect main knife switch
├── switch_description/           # Toggle switches (bistable On/Off states)
├── rotary_description/           # Continuous rotary potentiometer knobs
├── led_indicator_description/    # Multicolored status LEDs with dynamic illumination
├── socket_description/           # Electrical power and data sockets
├── row_description/              # Terminal strip block arrays
├── electromagnet_description/    # Magnetic latching mechanisms
├── aruco_description/            # Calibration and panel orientation ArUco targets
└── erc_panel_sim/                # Top-level URDF assembly, joint controllers & launch files
```

---

## 🦾 Joint Dynamics & Physics Interfaces

Each switch and dial is physically articulated in Gazebo Fortress using revolute/prismatic joints with realistic friction, restitution, and damping:

| Component | Joint Type | Motion Limits | Restoring Force / Damping |
| :--- | :--- | :--- | :--- |
| **Main Switch** | Revolute | $0.0 \rightarrow 1.57\,\text{rad}$ ($90^\circ$) | Spring-loaded detents |
| **Toggle Switches** | Revolute | $-0.52 \rightarrow +0.52\,\text{rad}$ ($\pm 30^\circ$) | Bistable snap-action |
| **Rotary Knobs** | Continuous | Unlimited ($0 \rightarrow 2\pi\,\text{rad}$) | Viscous friction damping |
| **Sockets** | Prismatic | $0.0 \rightarrow 0.04\,\text{m}$ insertion | Coulomb friction fit |
