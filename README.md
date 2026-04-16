# ROAR Workspace Old (ROS2 - MOST COMPLETE)

## Purpose
**MAIN ROS2 workspace** - Contains fully migrated roar_simulation package

## Status
- ✅ Package builds successfully
- ✅ Launch files migrated to Python
- ✅ Python scripts migrated (rospy → rclpy)
- ✅ URDF valid
- ✅ All backups preserved
- ❌ RViz rendering (WSL OpenGL issue)

## Launch Commands
```bash
cd ~/roar_workspace_old
source install/setup.bash

# View in RViz (after fixing graphics)
ros2 launch roar_simulation view_rover_rviz.launch.py

# Spawn in Gazebo
ros2 launch roar_simulation rover_spawn.launch.py
```

## Package
- roar_simulation (ROS2 Humble)
- Format 3, ament_cmake
