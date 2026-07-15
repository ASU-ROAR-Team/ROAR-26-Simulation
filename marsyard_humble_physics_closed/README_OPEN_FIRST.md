# Mars Yard ROS 2 Environment - Physics Closed Version

Target:
- Ubuntu 22.04
- ROS 2 Humble
- Ignition Gazebo Fortress

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch marsyard marsyard.launch.py
```

What this package contains:
- Mars Yard terrain only.
- No rover.
- No dummy rover.
- No Gazebo Classic.
- No `gz sim`.
- No flat ground plane under the Mars Yard.
- Gazebo grid disabled in the world scene.
- Textured visual terrain.
- Invisible low-poly collision terrain for rover climbing / descending.

Physics setup:
- World gravity is `9.81 m/s²` because ERC rover testing happens on Earth.
- Physics engine is DART through Ignition Fortress.
- `max_step_size = 0.004`, `real_time_update_rate = 250` for stable but not too heavy simulation.
- Terrain friction is `mu = 1.05`, `mu2 = 1.05` with small slip `0.015` to behave like compact sand / rough soil.

Important:
- The visible mesh is not used for physics.
- The invisible collision mesh is intentionally low-poly to avoid Gazebo / DART crashes.
- SDF is correct for worlds/environments. URDF/Xacro should be used later for the rover only.
