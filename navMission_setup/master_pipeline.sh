#!/bin/bash
set -e

echo "=========================================================="
echo "    STARTING FULL AUTOMATION PIPELINE"
echo "=========================================================="

# 1. World Generation & NavMission Setup
echo "[1/4] Building Worlds (1, 2, 3) and applying NavMission setup..."
cd $HOME/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/navMission_setup
./build_full_worlds.sh

# 2. Waypoint Generation & Mission Sync
echo "[2/4] Fusing Waypoints and Syncing to Mission Workspace..."
cd $HOME/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/navMission_setup/waypoints_setup/wayPoint_generator

for w in 1 2 3; do
    echo "  -> Processing waypoints for World $w..."
    
    # Clear old inputs
    rm -f inputs/heightmap.npz inputs/obstacle_data*.npy
    
    # Copy new assets to inputs
    # Wait, the outputs are cleared by build_full_worlds.sh at the end.
    # But they are copied to dev_environment/worlds/world$w/ !
    DEV_WORLD="$HOME/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/dev_environment/worlds/world$w"
    
    if [ -f "$DEV_WORLD/heightmap.npz" ]; then
        cp "$DEV_WORLD/heightmap.npz" inputs/heightmap.npz
    fi
    if [ -f "$DEV_WORLD/obstacle_data.npy" ]; then
        cp "$DEV_WORLD/obstacle_data.npy" inputs/obstacle_data.npy
    fi
    
    # Generate waypoints using the updated margins
    python3 wp_generator.py
    
    # Sync to Mission Workspace
    MISSION_DIR="$HOME/Navigation_Mission_Workspace/src/ercNavigation_Mission/missions/mission_$w/waypoints"
    mkdir -p "$MISSION_DIR"
    
    # The output is in outputs/wpXX.npy
    for wp_file in outputs/wp*.npy; do
        if [ -f "$wp_file" ]; then
            wp_name=$(basename "$wp_file" .npy)
            mkdir -p "$MISSION_DIR/$wp_name"
            cp "$wp_file" "$MISSION_DIR/$wp_name/$wp_name.npy"
        fi
    done
    
    # Also sync the costmap to the assets directory for the GUI preview!
    ASSETS_DIR="$HOME/Navigation_Mission_Workspace/src/ercNavigation_Mission/missions/mission_$w/assets"
    mkdir -p "$ASSETS_DIR"
    if [ -f "$DEV_WORLD/costmap.npz" ]; then
        cp "$DEV_WORLD/costmap.npz" "$ASSETS_DIR/costmap.npz"
    fi
done

echo "[3/4] Cleaning Setup Directories..."
rm -f inputs/heightmap.npz inputs/obstacle_data*.npy outputs/wp*.npy

echo "=========================================================="
echo " ✅ SUCCESS! Pipeline Complete!"
echo "=========================================================="
