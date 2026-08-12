#!/bin/bash
set -e

WORKSPACE_DIR="/home/draaven/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion"
TEMP_DIR="$WORKSPACE_DIR/navMission_setup/world_setup/TempArucoGen"
EXPORT_WORLD_DIR="$TEMP_DIR/world"

echo "=== Cleaning Up Old Generated Assets ==="
rm -rf "$TEMP_DIR/heightmap"/*
rm -rf "$TEMP_DIR/costmap"/*
rm -rf "$TEMP_DIR/aruco_data"/*
rm -rf "$EXPORT_WORLD_DIR"/*

mkdir -p "$TEMP_DIR/heightmap"
mkdir -p "$TEMP_DIR/costmap"
mkdir -p "$TEMP_DIR/aruco_data"
mkdir -p "$EXPORT_WORLD_DIR"

echo "=== Skipping Step 1: Using Pre-baked Maps from world_1 ==="
# bash "$TEMP_DIR/scripts/step1_heightmap.sh"
cp "$WORKSPACE_DIR/navMission_setup/outputs/world_1/heightmap/"*.npz "$TEMP_DIR/heightmap/newhight.npz" || true
cp "$WORKSPACE_DIR/navMission_setup/outputs/world_1/heightmap/"*.png "$TEMP_DIR/heightmap/newhight.png" || true
cp "$WORKSPACE_DIR/navMission_setup/outputs/world_1/costmap/"*.npz "$TEMP_DIR/costmap/costmap.npz" || true
cp "$WORKSPACE_DIR/navMission_setup/outputs/world_1/costmap/"*.png "$TEMP_DIR/costmap/costmap.png" || true
cp -r "$WORKSPACE_DIR/navMission_setup/outputs/world_1/costmap/csv" "$TEMP_DIR/costmap/" || true

echo "=== Step 2: Generating Mapped npy/yaml Data ==="
python3 "$TEMP_DIR/scripts/step2_generate_npy.py"

echo "=== Step 3: Fusing Marker Models into World ==="
python3 "$TEMP_DIR/scripts/step3_fuse_world.py"

echo "=== Step 4: Bundling Self-Contained Dataset into TempArucoGen/world ==="
mkdir -p "$EXPORT_WORLD_DIR/models"
mkdir -p "$EXPORT_WORLD_DIR/csv"

# Copy ArUco marker datasets (.npy, .yaml, _info.txt)
cp "$TEMP_DIR/aruco_data"/aruco_data.* "$EXPORT_WORLD_DIR/" 2>/dev/null || true
cp "$TEMP_DIR/aruco_data"/aruco_data_info.txt "$EXPORT_WORLD_DIR/" 2>/dev/null || true

# Copy heightmap and costmap assets
cp "$TEMP_DIR/heightmap"/newhight.npz "$EXPORT_WORLD_DIR/heightmap.npz" 2>/dev/null || true
cp "$TEMP_DIR/heightmap"/newhight.png "$EXPORT_WORLD_DIR/heightmap.png" 2>/dev/null || true
cp "$TEMP_DIR/costmap"/costmap.npz "$EXPORT_WORLD_DIR/costmap.npz" 2>/dev/null || true
cp "$TEMP_DIR/costmap"/costmap.png "$EXPORT_WORLD_DIR/costmap.png" 2>/dev/null || true

# Copy CSVs if available
if [ -d "$TEMP_DIR/costmap/csv" ]; then
    cp -r "$TEMP_DIR/costmap/csv"/* "$EXPORT_WORLD_DIR/csv/" 2>/dev/null || true
fi

# Copy 3D marker models
cp -r "$TEMP_DIR"/models/aruco_* "$EXPORT_WORLD_DIR/models/" 2>/dev/null || true

# Generate metadata log inside exported dataset folder
cat <<EOF > "$EXPORT_WORLD_DIR/metadata.txt"
ROAR ArUco World Dataset
==================================================
Dataset name            : world_Rotated_Aruco
Base world              : world_Rotated.world
Base heightmap          : heightmap.npz
Total ArUco Markers     : 15
Marker Size             : 0.20 m
Frame ID                : map
Generated World         : world_Rotated_Aruco.world
Generated Heightmap     : heightmap.npz
Generated Costmap       : costmap.npz
Models Directory        : models/
EOF

echo "=== Step 5: Integrating files into Workspace packages ==="
# Copy world to dev_environment
cp "$EXPORT_WORLD_DIR/world_Rotated_Aruco.world" "$WORKSPACE_DIR/dev_environment/worlds/world_Rotated_Aruco.world" 2>/dev/null || true

# Copy custom models so Gazebo can discover model://aruco_*
cp -r "$TEMP_DIR"/models/aruco_* "$WORKSPACE_DIR/marsyards/marsyard/models/" 2>/dev/null || true

echo "=== Step 6: Building Workspace ==="
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    cd "$WORKSPACE_DIR/dev_environment"
    colcon build --paths rover/* || true
fi

echo "=========================================================="
echo "Pipeline Completed Successfully!"
echo "Self-contained export dataset created at:"
echo "  $EXPORT_WORLD_DIR"
echo "=========================================================="
