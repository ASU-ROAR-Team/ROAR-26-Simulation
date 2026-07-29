#!/bin/bash
set -e

WORKSPACE_DIR="/home/saif/Desktop/ROAR/simulation_ws"
TEMP_DIR="$WORKSPACE_DIR/src/navMission_setup/world_setup/TempArucoGen"

echo "=== Cleaning Up Old Generated Assets ==="
rm -rf "$TEMP_DIR/heightmap"/*
rm -rf "$TEMP_DIR/costmap"/*
rm -rf "$TEMP_DIR/world"/*
rm -rf "$TEMP_DIR/aruco_data"/*

echo "=== Step 1: Generating Heightmap ==="
bash "$TEMP_DIR/scripts/step1_heightmap.sh"

echo "=== Step 2: Generating Mapped npy Data ==="
python3 "$TEMP_DIR/scripts/step2_generate_npy.py"

echo "=== Step 3: Fusing Marker Models into World ==="
python3 "$TEMP_DIR/scripts/step3_fuse_world.py"

echo "=== Step 4: Integrating files into Workspace packages ==="
# Copy world
cp "$TEMP_DIR/world/world_Rotated_Aruco.world" "$WORKSPACE_DIR/src/marsyards/worlds/worlds/world_Rotated_Aruco.world"

# Copy custom models to marsyard package so Gazebo can discover model://aruco_*
cp -r "$TEMP_DIR"/models/aruco_* "$WORKSPACE_DIR/src/marsyards/marsyard/models/"

echo "=== Step 5: Building Workspace ==="
source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"
colcon build --packages-select marsyard worlds --symlink-install

echo "=== Pipeline Completed Successfully! ==="
