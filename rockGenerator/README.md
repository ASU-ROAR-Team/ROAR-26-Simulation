# Rock Generator and SDF Environment Models

This directory contains the assets, models, and generator tools used to spawn random rock obstacles inside the ROAR Mars Yard Gazebo simulation.

---

## 1. How to Use the Rock Generator

The `rock_generator.py` script automatically generates a set of rocks in the active simulation. It places them at a specific height and lets them fall under gravity to settle naturally on the terrain slopes. 

### Prerequisites
- Gazebo simulation must be running (either Closed Physics or Final Yard).
- **The simulation must be played/unpaused** (clock is actively running).

### Running the Generator
```bash
# Navigate to this directory
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator

# Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash

# Run the generator script
python3 rock_generator.py
```

### Configurable Parameters
You can edit the parameters directly in the top block of `rock_generator.py` or override them dynamically using CLI flags.

| Parameter / CLI Flag | Default Value | Description |
| :--- | :--- | :--- |
| `X_RANGE` / `--x-range` | `(-8.0, 8.0)` | Spawning boundary along the X-axis (min, max) in meters (e.g. `--x-range -15 15` for the whole map). |
| `Y_RANGE` / `--y-range` | `(-8.0, 8.0)` | Spawning boundary along the Y-axis (min, max) in meters (e.g. `--y-range -15 15` for the whole map). |
| `SPAWN_Z` | `4.0` | Initial height to drop rocks from. Ensures they settle on terrain and do not spawn underground. |
| `NUM_ROCKS` / `-n`, `--num-rocks` | `15` | Total number of rocks to generate (overall density). |
| `GROUP_1_RATIO` / `-g1`, `--group1-ratio` | `0.6` | Ratio/probability of choosing a Group 1 rock (rock_1 to rock_5) vs a Group 2 rock (rock_6 to rock_9) (e.g., 0.6 = 60% Group 1). |
| `GROUP_1_COLLIDABLE_RATIO` / `-c1`, `--g1-collidable-ratio` | `0.7` | Ratio of Group 1 rocks to be collidable (e.g., 0.7 = 70% solid, 30% ghost). Group 2 rocks are always non-collidable (ghosts). |
| `FALL_WAIT_TIME` | `2.0` | Time (seconds) allowed for rocks to fall and settle before they are frozen in position. |
| `FREEZE_COLLIDABLE` | `True` | If `True`, solid rocks will be frozen (converted to static) once settled, preventing the rover from pushing them. |
| `GENERATE_DEADENDS` / `--deadends`, `--no-deadends` | `False` | Spawns a structured barrier of collidable rocks at Y=2.5 blocking the main central path to create deadends. |
| `SPAWN_DELAY` / `-d`, `--spawn-delay` | `2.0` | Time delay (cooldown in seconds) between spawning each rock to prevent simulation physics lag. |
| `FREEZE_DELAY` | `0.5` | Time delay (seconds) between freezing each rock to ensure Gazebo handles entity swapping stably. |

### Advanced CLI Launching Examples
You can run the script with customized parameters directly from the command line:

```bash
# Run with a higher density of 30 rocks, a spawn cooldown of 0.5s, and custom group ratios
python3 rock_generator.py --num-rocks 30 --spawn-delay 0.5 --group1-ratio 0.5 --g1-collidable-ratio 0.8

# Spawn rocks across the entire 30m x 30m map bounds
python3 rock_generator.py --num-rocks 25 --x-range -15.0 15.0 --y-range -15.0 15.0 --spawn-delay 0.8

# Run and generate a deadend barrier
python3 rock_generator.py --deadends --num-rocks 15 --spawn-delay 1.0
```

---

## 2. Anatomy of the Rock SDF Files

Each rock model is stored under `rocks_ws/rock_<id>/`. They contain standard Gazebo models structured using the **Simulation Description Format (SDF)**.

Here is the default content of `rocks_ws/rock_1/model.sdf`:
```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="rock_1">
    <static>true</static>
    <link name="rock_link">
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>meshes/rock_1.dae</uri>
          </mesh>
        </geometry>
      </visual>

      <collision name="collision">
        <geometry>
          <mesh>
            <uri>meshes/rock_1.stl</uri>
          </mesh>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
```

### Key Components Explained:

1. **`<model name="rock_1">`**:
   Defines the unique name of the model entity in Gazebo. When spawning rocks, the script overrides this name dynamically (e.g., `temp_fall_Rock 1`, `Rock 1`) to avoid name collisions.

2. **`<static>true</static>`**:
   - `true`: The model is static (immovable, unaffected by gravity, forces, or collisions).
   - `false`: The model is dynamic (falls under gravity, subject to physics forces, and behaves as a rigid body).

3. **`<visual name="visual">`**:
   Represents what is rendered on screen. It references a **`.dae` (COLLADA)** file inside `meshes/`. DAE meshes contain full color, textures, UV mappings, and materials (extracted from the original `.glb` models). This resolves the issue of rocks appearing as solid flat white models.

4. **`<collision name="collision">`**:
   Represents the physical boundaries used by the physics engine (DART/ODE) to calculate collisions. It references a **`.stl`** file inside `meshes/` for precise vertex-level collision testing. Using STL for collision keeps the collision geometry lightweight, optimizing physics computations.

---

## 3. How It Works (The Spawning Pipeline)

To spawn a mixture of solid and ghost rocks, the generator script reads the base `model.sdf` file, parses its XML, and writes a temporary modified SDF file in the rock's folder before calling the spawn service:

- **For Dynamic Falling (Physics-Settle Phase)**:
  - `<static>` is set to `false`.
  - An `<inertial>` block with a `<mass>` of `5.0 kg` and simple inertia matrices (`ixx`, `iyy`, `izz` = `0.1`) is injected into the `<link>` to satisfy the rigid-body physics solver.
  - The `<collision>` element is kept intact so that the rock lands on the ground.

- **For Permanent Spawning (Static Phase)**:
  - `<static>` is set to `true` (making it immovable).
  - **For Ghost Rocks**: The `<collision>` tag is completely removed from the XML so that the rover can drive through the visual model.
  - **For Solid Rocks**: The `<collision>` tag is kept so that the physics engine blocks any rover contact.

---

## 4. How to Spawn Individual Rocks Manually

If you want to inspect or test individual rock models one by one in the Gazebo simulation without running the generator script, you can spawn them directly from the terminal.

### Step 1: Source the ROS 2 Environment
Open a terminal and ensure ROS 2 Humble is sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Step 2: Spawn a Selected Rock
Use the `ros_gz_sim` spawner node (`ros2 run ros_gz_sim create`) to load the SDF. For example, to spawn **Rock 1** at coordinates `(X=0.0, Y=0.0, Z=2.0)`:
```bash
ros2 run ros_gz_sim create \
  -file /home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator/rocks_ws/rock_1/model.sdf \
  -name "Rock 1 inspect" \
  -x 0.0 -y 0.0 -z 2.0
```
*(You can change `rock_1` in the path to any other rock folder from `rock_1` to `rock_9`, and change the `-name` argument to keep it unique).*

### Step 3: Remove the Spawned Rock
Once you are done inspecting the rock, you can clean it up by calling the Gazebo entity removal service:
```bash
ign service -s /world/marsyard/remove \
  --reqtype ignition.msgs.Entity \
  --reptype ignition.msgs.Boolean \
  --timeout 1000 \
  --req 'name: "Rock 1 inspect", type: 2'
```
*(If the `ign` CLI is not found on your system, use `gz` instead of `ign` at the start of the command).*
