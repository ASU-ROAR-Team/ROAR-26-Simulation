# Marsyard Orientation & Coordinate System Solution

This file explains the orientation issue identified with the Marsyard terrain model and how it was successfully resolved inside the simulator.

---

## 1. The Problem

Initially, the simulated Marsyard world orientation did not match the official coordinate system shown in `MarsYard_CoordSys.PNG` or the coordinates listed in the `[ERC 2025] MY Update Report Rev.2.pdf`.
* **Incorrect Orientation**: The wedge (funnel extension) of the yard was located at the **top-left** instead of the **bottom-right** (as pictured in the reference map).
* **Mismatch in Coordinates**: Because the orientation was flipped and rotated, inputting direct $(x, y)$ coordinate positions from the PDF report resulted in incorrect placements of objects and target waypoints relative to the terrain features.

---

## 2. The Solution

To solve this without manually modifying the raw 3D mesh OBJ assets, we resolved the mirroring and rotation programmatically within the simulation configuration.

### A. Fixing the Mirroring (Layout Reflection)
If we simply rotated the model by 180 degrees, it would not correct a layout reflection (mirroring) issue caused by coordinate system handedness. Attempting to mirror using a pitch/roll rotation (like `roll 3.14`) flipped the ground upside down, pointing the visual terrain normal towards the void below.
* **The Fix**: We mirrored the meshes safely by applying a scaling factor of `<scale>1 -1 1</scale>` (mirroring along the Y-axis) directly inside the visual and collision `<mesh>` configurations of the model SDF. This keeps the ground normal pointing UP correctly.

### B. Fixing the Rotation (Orienting the Wedge)
To place the wedge at the bottom-right and align the terrain view with the coordinate photo, the mirrored mesh required a 90-degree yaw rotation.
* **The Fix**: We embedded the yaw rotation of `1.5708` ($\pi/2$ radians) directly inside the model's `<link name="terrain_link">` description. By fusing it into the link element of the model, the simulation world files can import the model using a clean base pose:
  ```xml
  <pose>0 0 0 0 0 0</pose>
  ```

---

## 3. What Was Edited

We updated the workspace folders to achieve this:
1. **Model Directory (`marsyard` package)**:
   - Modified [src/marsyards/marsyard/models/mars_yard/model.sdf](file:///home/saif/Desktop/ROAR/simulation_ws/src/marsyards/marsyard/models/mars_yard/model.sdf) to include the Y-axis scale mirroring and link pose yaw.
   - Modified [src/dev_environment/models/mars_yard/model.sdf](file:///home/saif/Desktop/ROAR/simulation_ws/src/dev_environment/models/mars_yard/model.sdf) similarly.
2. **Worlds Directory (`worlds` package)**:
   - Modified the standard worlds to reset the model pose coordinates to zero:
     - [src/marsyards/worlds/worlds/marsyard.world](file:///home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/worlds/marsyard.world)
     - [src/marsyards/marsyard/worlds/marsyard.world](file:///home/saif/Desktop/ROAR/simulation_ws/src/marsyards/marsyard/worlds/marsyard.world)
   - Created the new rotated world [src/marsyards/worlds/worlds/world_Rotated.world](file:///home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/worlds/world_Rotated.world) and its launch script [src/marsyards/worlds/launch/world_Rotated.launch.py](file:///home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/launch/world_Rotated.launch.py).
