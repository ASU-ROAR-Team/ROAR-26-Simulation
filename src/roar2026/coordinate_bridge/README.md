# Coordinate Bridge

Centralized coordinate conversion between the Mars Yard reference coordinate system and the Gazebo coordinate system.

## Purpose

The Mars Yard terrain is already visually correct in Gazebo and must **not** be modified.

The reference/navigation coordinate system uses the opposite Y direction:

```text
Reference → Gazebo

X =  X
Y = -Y
Z =  Z
```

The terrain mesh, collision mesh, and Gazebo world frame are therefore left unchanged.

The coordinate bridge provides one centralized place for converting coordinates between the two conventions.

---

## Coordinate Convention

### Reference frame

```text
        +Y
        ↑
        |
        |
        +------→ +X
```

### Gazebo/reference relationship

```text
x_gazebo = x_reference
y_gazebo = -y_reference
z_gazebo = z_reference
```

The inverse conversion is identical:

```text
x_reference = x_gazebo
y_reference = -y_gazebo
z_reference = z_gazebo
```

Yaw is also reversed:

```text
yaw_gazebo = -yaw_reference
yaw_reference = -yaw_gazebo
```

---

## Package Structure

```text
coordinate_bridge/
├── coordinate_bridge/
│   ├── __init__.py
│   └── converter.py
├── package.xml
├── setup.py
└── README.md
```

`converter.py` contains the coordinate conversion functions.

---

## Installation

From the simulation workspace:

```bash
cd ~/Simulation_ws
colcon build --symlink-install --packages-select coordinate_bridge
source install/setup.bash
```

---

## Using the Converter

Import the conversion functions:

```python
from coordinate_bridge.converter import (
    ref_to_gazebo_position,
    gazebo_to_ref_position,
    ref_to_gazebo_yaw,
    gazebo_to_ref_yaw,
)
```

### Reference → Gazebo

```python
gx, gy, gz = ref_to_gazebo_position(
    ref_x,
    ref_y,
    ref_z,
)

gazebo_yaw = ref_to_gazebo_yaw(reference_yaw)
```

### Gazebo → Reference

```python
rx, ry, rz = gazebo_to_ref_position(
    gazebo_x,
    gazebo_y,
    gazebo_z,
)

reference_yaw = gazebo_to_ref_yaw(gazebo_yaw)
```

---

# `.world` Handoff to Another Repository

The `.world` file itself is a **Gazebo artifact**.

The other repository does **not** need the coordinate-bridge package merely to load the terrain world.

For example:

```text
Mars Yard repository
        │
        │
        ├── coordinate_bridge
        │
        └── marsyard.world
                  │
                  │ copy
                  ▼
        Navigation / Simulation repository
                  │
                  ▼
                Gazebo
```

## Important distinction

The coordinate bridge is required wherever **reference coordinates are being converted into Gazebo coordinates**.

The `.world` file does not automatically perform Python conversions.

Therefore, if Mars Yard generates object poses before producing the final `.world`, those poses should be converted **before being written into the `.world`**.

For example:

```python
gx, gy, gz = ref_to_gazebo_position(
    reference_x,
    reference_y,
    reference_z,
)

pose = f"{gx} {gy} {gz} 0 0 {gazebo_yaw}"
```

The resulting `.world` contains ordinary SDF:

```xml
<pose>10.0 -5.0 0.2 0 0 -1.57</pose>
```

Once these values are written into the `.world`, the receiving repository does not need to know how they were generated.

---

# If the Other Repository Spawns Objects

There is one important exception.

If the other repository loads:

```text
marsyard.world
```

and then separately spawns:

* rover
* ArUco markers
* obstacles
* other models

using reference coordinates, those spawn coordinates must also pass through the coordinate bridge.

For example:

```text
Reference rover pose
        │
        ▼
coordinate_bridge
        │
        ▼
Gazebo rover pose
```

The same applies to ArUco markers and obstacles.

The goal is that **every coordinate crossing into Gazebo uses the same conversion**.

---

# What Must NOT Be Changed

Do not modify the Mars Yard mesh to solve this coordinate issue.

Do not use:

```xml
<scale>1 -1 1</scale>
```

Do not rotate the entire terrain to compensate for the coordinate convention.

Do not modify:

```xml
<pose>0 0 0 0 0 0</pose>
```

of the Mars Yard terrain merely to fix the Y convention.

The terrain visualization is already correct.

---

# Verification

Test the converter:

```bash
python3 - <<'PY'
from coordinate_bridge.converter import ref_to_gazebo_position

print(ref_to_gazebo_position(10, 5, 2))
print(ref_to_gazebo_position(-3, -7, 1))
PY
```

Expected:

```text
(10, -5, 2)
(-3, 7, 1)
```

---

# Responsibility Between Repositories

### Mars Yard repository

Responsible for:

* Mars Yard terrain
* terrain `.world`
* coordinate conversion utilities
* converting reference object poses before baking them into generated SDF/world files

### Receiving simulation/navigation repository

Responsible for:

* loading the provided `.world`
* converting any **new runtime-spawned reference coordinates**
* using the same coordinate convention for rover, ArUco, obstacles, waypoints, etc.

The `.world` itself remains a normal, self-contained Gazebo SDF file.

---

# Important Limitation

A standard ROS TF transform cannot represent:

```text
X → X
Y → -Y
Z → Z
```

because this is a reflection rather than a rigid right-handed transform.

Therefore this package is a **coordinate conversion layer**, not a replacement for a TF frame.

It should be used at the boundary where coordinates enter or leave the Gazebo coordinate system.

---

# Example End-to-End Flow

```text
Reference / Navigation Data
          │
          │
          ▼
   coordinate_bridge
          │
          │ X = X
          │ Y = -Y
          │ Z = Z
          ▼
   Gazebo Coordinates
          │
          ├── Rover
          ├── ArUco
          ├── Obstacles
          ├── Waypoints
          └── Terrain
```

The Mars Yard terrain remains visually unchanged while all externally supplied object coordinates use one consistent conversion.

