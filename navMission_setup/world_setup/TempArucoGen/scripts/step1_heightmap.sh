#!/bin/bash
set -e

# Paths
WORKSPACE_DIR="/home/draaven/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion"
WORLD_PATH="$WORKSPACE_DIR/navMission_setup/outputs/world_1/world/world_1_d0.008_c0.50.world"
TEMP_DIR="$WORKSPACE_DIR/navMission_setup/world_setup/TempArucoGen"

HEIGHTMAP_GEN="$TEMP_DIR/scripts/heightmap_generator.py"
COSTMAP_GEN="$TEMP_DIR/scripts/costmap_generator.py"

OUT_HEIGHT_NPZ="$TEMP_DIR/heightmap/newhight.npz"
OUT_HEIGHT_PNG="$TEMP_DIR/heightmap/newhight.png"

OUT_COST_NPZ="$TEMP_DIR/costmap/costmap.npz"
OUT_COST_PNG="$TEMP_DIR/costmap/costmap.png"

echo "=== Sourcing Environment (if available) ==="
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
fi

echo "=== Step 1a: Running Heightmap Generator ==="
python3 "$HEIGHTMAP_GEN" "$WORLD_PATH" -o "$OUT_HEIGHT_NPZ" --preview "$OUT_HEIGHT_PNG"

echo "=== Step 1b: Running Costmap Generator ==="
python3 "$COSTMAP_GEN" "$OUT_HEIGHT_NPZ" -o "$OUT_COST_NPZ" --preview "$OUT_COST_PNG"

echo "=== Maps generated successfully! ==="
