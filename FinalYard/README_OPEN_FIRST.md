# Mars Yard exact-footprint package — runtime world fix

Target: Ubuntu 22.04 + ROS 2 Humble + Ignition Gazebo / Fortress.

## Run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch marsyard marsyard.launch.py
```

## What this fixed

This version does not depend on Gazebo guessing `model://mars_yard`.
The launch file writes `/tmp/marsyard_runtime.world` and includes the model by absolute `file://.../model.sdf`.

It also removes the custom GUI block that was causing a blank-looking 3D view on some Humble/Fortress setups.


This v4 variant closes the visible edges with a side skirt / bottom cap and slightly trims the footprint boundary to remove ragged outer artifacts.
