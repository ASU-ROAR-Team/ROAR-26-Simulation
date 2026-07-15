# ROAR Mars Yard Simulation Workspace

This workspace contains the Gazebo simulation environments for the Mars Yard along with the complete ROAR Rover (with robotic arm) simulation.

---

## 1. Directory Structure

- **`ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/`**: ROS 2 package for the Rover and Arm model (`roar_simulation`).
- **`marsyard_humble_physics_closed/`**: ROS 2 workspace containing the closed-physics Mars Yard world (`marsyard`).
- **`FinalYard/`**: ROS 2 workspace containing the high-fidelity Final Mars Yard world (`marsyard`).
- **`rockGenerator/`**: Simulation utility for spawning random and structured rocks inside the active Gazebo environment.
- **`gui_teleop.py`**: A standalone, rover/world-agnostic Tkinter GUI teleoperation panel.
- **`data/`**: Storage folder for other assets, logs, and photos.

---

## 2. Step-by-Step Run Guide

Follow these steps sequentially to build the workspaces, launch a world, spawn the rover, and control it.

### Step 0: Clean Up Previous Runs
Before initiating a new run, ensure all lingering processes are terminated:
```bash
pkill -f -9 "ign|gazebo|ros2|parameter_bridge" || true
```

### Step 1: Build the Rover Model Workspace
First, we must compile the rover description package (`roar_simulation`):
```bash
# Navigate to the rover model workspace
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion

# Build and source
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 2: Build and Launch the Desired World Workspace
Choose **one** of the two worlds to launch:

#### Option A: Closed-Physics Mars Yard
This world features stabilized physics parameters for the standard Mars Yard terrain.
```bash
# In a new terminal, navigate to the closed-physics workspace
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/marsyard_humble_physics_closed

# Build and source (sourcing the rover workspace overlay first)
source /opt/ros/humble/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch the world
ros2 launch marsyard marsyard.launch.py
```

#### Option B: Final Mars Yard
This world contains the high-fidelity textured terrain with edge visual artifacts resolved.
```bash
# In a new terminal, navigate to the final yard workspace
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard

# Build and source (sourcing the rover workspace overlay first)
source /opt/ros/humble/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash
colcon build --symlink-install
source install/setup.bash

# Launch the world
ros2 launch marsyard marsyard.launch.py
```

---

### Step 3: Play/Unpause the Simulation
Before spawning the robot, you **must** play/unpause the simulator so the clock starts running:
- **Option 1 (GUI)**: Click the **Play** button in the bottom-left corner of the Gazebo window.
- **Option 2 (CLI)**: In a new terminal, run:
  ```bash
  ign service -s /world/marsyard/control --reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'pause: false'
  ```

---

### Step 4: Spawn the Rover with Robotic Arm
Once the simulation clock is ticking, spawn the rover in a separate terminal:
```bash
# Open a new terminal and source the build chain
source /opt/ros/humble/setup.bash
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash

# Source the active world workspace (change FinalYard to marsyard_humble_physics_closed if you chose Option A)
source /home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard/install/setup.bash

# Spawn the rover (with controllers and bridges)
ros2 launch marsyard spawn_robot.launch.py \
  robot_name:=roar_rover \
  urdf_path:=$(ros2 pkg prefix roar_simulation)/share/roar_simulation/urdf/roar_complete_sim.urdf.xacro \
  x:=0.0 y:=0.0 z:=4.0 yaw:=0.0
```

*Note: Spawning the robot while the clock is active allows the ROS 2 controllers to bind and transition to the `active` state automatically.*

---

### Step 5: Control the Rover with the GUI
Control the rover using the agnostic GUI teleoperation panel.

```bash
# Open a new terminal
cd /home/saif/Desktop/ROAR/MARS_YARD_INIT

# Source the active ROS 2 environment
source /opt/ros/humble/setup.bash

# Run the GUI script (defaulting to /cmd_vel)
python3 gui_teleop.py
```

#### Agnostic Config Options:
- **Custom Topic via CLI**: Run `python3 gui_teleop.py --topic=/model/roar_rover/cmd_vel` to start publishing to a specific name.
- **Dynamic Topic Change in GUI**: Enter any custom topic path (e.g. `/cmd_vel`) in the **ROS Topic Config** input field and click **Apply** to dynamically switch the publisher's target.

---

### Step 6: Spawn Random Rocks in the World
To generate random rocks of varying collidability, density, and positions:

1. **Configure Parameters**:
   Open `/home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator/rock_generator.py` and modify the values in the `CONFIGURABLE PARAMETERS` block at the top of the file:
   * `X_RANGE`, `Y_RANGE`: The boundaries of the rock spawning region.
   * `NUM_ROCKS`: Total count of rocks to place in the world.
   * `COLLIDABLE_RATIO`: Probability (`0.0` to `1.0`) of a rock being collidable (solid obstacle) vs. non-collidable (ghost model).
   * `FALL_WAIT_TIME`: Time allowed (in seconds) for rocks to fall and settle on the terrain before their positions are locked.
   * `FREEZE_COLLIDABLE`: Freeze collidable rocks into permanent, immovable static obstacles once they settle (so they cannot be moved by the rover).

2. **Run the Generator**:
   Ensure the Gazebo simulation is running and **unpaused** (clock is ticking). In a new terminal, run:
   ```bash
   # Navigate to rockGenerator folder
   cd /home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator

   # Source ROS 2 environment
   source /opt/ros/humble/setup.bash

   # Run the generator
   python3 rock_generator.py
   ```

---

### Step 7: Spawn Specific Rocks Manually (Optional)
If you want to spawn individual rock models one by one to inspect them:
1. **Source the ROS 2 Environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Spawn a selected rock** (e.g. `rock_1` at coordinates X=0.0, Y=0.0, Z=2.0):
   ```bash
   ros2 run ros_gz_sim create \
     -file /home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator/rocks_ws/rock_1/model.sdf \
     -name rock_1_inspect \
     -x 0.0 -y 0.0 -z 2.0
   ```
   *(To inspect other models, change `rock_1` to `rock_2`... `rock_9` in the path, and give it a unique `-name` value).*
3. **Remove the spawned rock** when done:
   ```bash
   gz service -s /world/marsyard/remove \
     --reqtype ignition.msgs.Entity \
     --reptype ignition.msgs.Boolean \
     --timeout 1000 \
     --req 'name: "rock_1_inspect", type: 2'
   ```

---

## 3. Running Everything from the Workspace Root

If you want to build and launch everything directly from the workspace root folder (`/home/saif/Desktop/ROAR/MARS_YARD_INIT`) without manually changing directories:

### Step 1: Build the Rover Model
```bash
# Source ROS 2 and build the rover description workspace
source /opt/ros/humble/setup.bash
colcon build --base-paths ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion --symlink-install
```

### Step 2: Build and Launch the Final Yard World
```bash
# In a new terminal, build and run the FinalYard package
(cd FinalYard && ./run_marsyard.sh)
```
*(Make sure to click Play/Unpause in Gazebo or run Section 2, Step 3's pause: false command before proceeding)*

### Step 3: Spawn the Rover with Robotic Arm
```bash
# In a new terminal, spawn the rover using the compiled package paths
source /opt/ros/humble/setup.bash
source ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/install/setup.bash
source FinalYard/install/setup.bash

ros2 launch marsyard spawn_robot.launch.py \
  robot_name:=roar_rover \
  urdf_path:=$(ros2 pkg prefix roar_simulation)/share/roar_simulation/urdf/roar_complete_sim.urdf.xacro \
  x:=0.0 y:=0.0 z:=4.0 yaw:=0.0
```

### Step 4: Run the Rock Generator
```bash
# In a new terminal, run the rock generator targeting the active simulation
source /opt/ros/humble/setup.bash
python3 rockGenerator/rock_generator.py
```

### Step 4b: Spawn Specific Rocks Manually (Alternative)
```bash
# In a new terminal, spawn an individual rock to inspect it
source /opt/ros/humble/setup.bash
ros2 run ros_gz_sim create \
  -file /home/saif/Desktop/ROAR/MARS_YARD_INIT/rockGenerator/rocks_ws/rock_1/model.sdf \
  -name rock_1_inspect \
  -x 0.0 -y 0.0 -z 2.0
```

### Step 5: Launch the Teleoperation GUI
```bash
# In a new terminal, run the control panel GUI
source /opt/ros/humble/setup.bash
python3 gui_teleop.py
```

