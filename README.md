# 🚀 ROAR 26 Simulation — Rover + 6-DOF Arm on Marsyard

> A comprehensive ROS 2 Humble simulation of the ROAR rover equipped with a 6-DOF robotic arm. Operating inside the Gazebo Fortress engine, this package drops the rover into the Marsyard 2024 environment with fully simulated sensors, ros2_control hardware interfaces, and environmental degradation (noise, dust, glare).
> 
> Part of **ASU ROAR Team's ERC 2026** preparation.

---

## 🏗️ System Architecture & Data Flow

This simulation bridges two main systems: the **Ignition Gazebo (Fortress)** physics engine, and the **ROS 2 Humble** middleware. 

### 1. The ROS 2 ↔ Gazebo Bridge
Because Gazebo uses its own transport system (`ignition.msgs`), we use the `ros_gz_bridge` to seamlessly translate messages to standard ROS 2 topics. 
- **Time Sync:** Gazebo provides the clock (`/clock`) to ensure ROS 2 nodes operate perfectly in sync with the physics engine.
- **Sensors:** Gazebo generates the raw camera matrices and IMU physics, which the bridge translates into standard `sensor_msgs`.
- **Transforms:** Gazebo's ground truth pose is broadcasted and converted into `tf2_msgs/msg/TFMessage` for precise internal tracking.

### 2. ROS 2 Control (Hardware Interfaces)
The URDF utilizes the `ign_ros2_control` plugin. This bypasses topic-level bridging for mechanical joints, providing direct shared-memory access between Gazebo physics and the `controller_manager`.
- The **`diff_drive_controller`** handles skid-steer/differential kinematics for the wheels.
- The **`joint_trajectory_controller`** handles smooth splines for the 6-DOF robotic arm.
- The **`joint_state_broadcaster`** queries all active joints and publishes them to `/joint_states`.

### 3. Core ROS 2 Topics
To interact with the robot, these are the critical topics:

**Rover Control & Odometry:**
- 🟢 **`/cmd_vel`** (`geometry_msgs/msg/Twist`): Target velocity commands for the rover base.
- 🟢 **`/diff_drive_controller/odom`** (`nav_msgs/msg/Odometry`): Live odometry estimation from wheel encoders.
- 🟢 **`/joint_states`** (`sensor_msgs/msg/JointState`): Real-time positions and velocities of all mechanical links.
  - *Rover Joints:* `wheel_rhs_front_joint`, `wheel_rhs_mid_joint`, `rocker_lhs_joint`, etc.
  - *Arm Joints:* `joint_1` through `joint_6`, plus end-effector joints (`joint_lee`, `joint_ree`).

**Arm Control:**
- 🟢 **`/arm_controller/joint_trajectory`** (`trajectory_msgs/msg/JointTrajectory`): Publish a list of joint positions, velocities, and timestamps to move the 6-DOF arm.
- 🟢 **`/arm_controller/state`** (`control_msgs/msg/JointTrajectoryControllerState`): Closed-loop feedback indicating current arm error and position.

**Perception & Sensors:**
- 📷 **`/zed2i/image_raw`** & **`/zed2i/depth`**: The pure, unaffected camera outputs direct from Gazebo.
- 🌫️ **`/zed2i/image_raw_updated`** & **`/zed2i/depth_updated`**: The final degraded sensor outputs simulating actual Marsyard conditions. Use these for testing robust perception algorithms.
- 🧭 **`/bno055/data`** (`sensor_msgs/msg/Imu`): Acceleration, angular velocity, and orientation data.

---

## 🚀 Launching Procedures & Boot Sequence

Before starting, ensure your workspace is compiled and sourced:
```bash
cd ~/roar_ws 
colcon build --packages-select roar_simulation
source install/setup.bash
```

We provide two distinct launch sequences depending on the algorithm you are testing:

### 1. "Clean" Simulation (Ideal Sensors)
Best for testing pure navigation algorithms, path planning, and kinematics without environmental interference.
```bash
ros2 launch roar_simulation basic_rover_clean.launch.py start_rviz:=true
```

### 2. "Noise" Simulation (Mars Conditions)
Includes the `zed_degradation_node` which injects environmental degradation (dust, distance-based depth decay, and dynamic sun glare). **This is the recommended environment for testing the final stack.**
```bash
ros2 launch roar_simulation basic_rover.launch.py start_rviz:=true
```

### ⏱️ The Boot Sequence
When you execute a launch file, a careful synchronization sequence occurs to prevent `ros2_control` crashes:
1. **0s**: Gazebo Server starts headless.
2. **3s**: Gazebo GUI Client launches and connects to the server.
3. **5s**: The rover model (`roar_rover`) and the `sun_marker` are spawned into the world.
4. **17s** *(12s after spawn)*: The `joint_state_broadcaster` is activated.
5. **18s**: The `diff_drive_controller` and `arm_controller` are spooled up.
6. **38s** *(20s after controllers)*: RViz automatically opens (if `start_rviz:=true`).

---

## 🎮 Controlling the Simulation

### 🚗 Driving the Rover (Teleop)
You can command the differential drive by running the custom teleop script:
```bash
ros2 run roar_simulation teleop.py
```
This will open a small Tkinter GUI window. **You must click the GUI window to focus it.** Once focused, use the **Arrow Keys** (Up, Down, Left, Right) to drive the rover. When you release a key, the rover will stop automatically. 

Alternatively, you can publish a Twist message manually:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

### 🦾 Controlling the Arm
The 6-DOF arm requires timed trajectory points. While you can technically publish to `/arm_controller/joint_trajectory` via the terminal, the expected methodology is to use **MoveIt2**. MoveIt2 will calculate Inverse Kinematics (IK) and safely path-plan collision-free trajectories directly into this topic.

---

## 🌪️ Environmental Degradation (The Noise Node)

When using `basic_rover.launch.py`, the `zed_degradation_node.py` runs in the background. It intercepts the clean Gazebo camera streams, applies numpy/OpenCV math, and outputs to the `*_updated` topics.

### What it injects:
1. **Martian Dust**: A stochastic mask applies a 1.5% coverage of brownish/orange pixels (`BGR: 80, 120, 180`) across the RGB and Depth images to simulate lens dirt.
2. **Quadratic Depth Error**: Calculates a standard deviation of $0.003 \times Z^2$. The further away an object is, the exponentially worse the depth reading becomes.
3. **Dynamic Sensor Blinding (Sun Glare)**: Calculates the exact 3D vector between the camera lens and the sun. If the sun enters the camera's FOV:
   - *RGB:* A massive white bloom effect washes out the image.
   - *Depth:* Pixels within the glare radius are converted to pure `NaN`, blinding depth perception.

### ☀️ How to Edit the Sun Position
The dynamic glare relies on a Ground-Truth Transform calculation between the `roar_rover` and an invisible `sun_marker` entity. It does **not** rely on the visual Gazebo sky sun.

If you modify the visual sun's position in the `.sdf` world file, you **must also move the `sun_marker`** so the glare matches the new light direction. 
To do this, edit `src/roar_simulation/launch/basic_rover.launch.py` (around line 102):

```python
# Edit the -x, -y, and -z values to place the invisible sun_marker 
# exactly where the visual sun is located in your world.
spawn_sun = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=['-file', sun_marker_file,
               '-name', 'sun_marker', 
               '-x', 'NEW_X', '-y', 'NEW_Y', '-z', 'NEW_Z'],
    ...
)
```

---

## ⚙️ Configuration Files Directory

All core tuning parameters are housed in `src/roar_simulation/config/`:

| File | Purpose |
|------|---------|
| `diff_controller.yaml` | Wheel separation distance, wheel radius, and base velocity limits. |
| `arm_controller.yaml` | Trajectory execution tolerances and joint PID mapping for the 6-DOF arm. |
| `combined_controllers.yaml` | Top-level `controller_manager` configuration detailing update rates (e.g., 100Hz). |
| `joint_names_rover.yaml` | Defines the specific rocker-bogie and wheel joint names from the URDF. |
| `joint_names_arm.yaml` | Defines `joint_1` through `joint_6` and the end-effector joints. |

---

## 👁️ Visualization (RViz)

If you didn't append `start_rviz:=true` during launch, you can boot RViz separately at any time with our pre-configured view:
```bash
ros2 launch roar_simulation view_rover_rviz.launch.py
```
This loads `basic_rover.rviz`, which has the appropriate `TF`, `Odometry`, `RobotModel`, and Camera plugins pre-configured to observe the simulated environment.
