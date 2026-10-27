#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

echo "=========================================================="
echo "    STARTING FULL AUTOMATION PIPELINE (MONOREPO)"
echo "=========================================================="

# 1. World Generation & NavMission Setup
echo "[1/4] Building Worlds (1, 2, 3) and applying NavMission setup..."
cd "${SCRIPT_DIR}"
./build_full_worlds.sh

# 2. Waypoint Generation & Mission Sync
echo "[2/4] Fusing Waypoints and Syncing to Mission Workspace..."
cd "${SCRIPT_DIR}/waypoints_setup/wayPoint_generator"

for w in 1 2 3; do
    echo "  -> Processing waypoints for World $w..."
    
    rm -f inputs/heightmap.npz inputs/obstacle_data*.npy
    DEV_WORLD="${SIM_DIR}/dev_environment/worlds/world$w"
    
    if [ -f "$DEV_WORLD/heightmap.npz" ]; then
        cp "$DEV_WORLD/heightmap.npz" inputs/heightmap.npz
    fi
    if [ -f "$DEV_WORLD/obstacle_data.npy" ]; then
        cp "$DEV_WORLD/obstacle_data.npy" inputs/obstacle_data.npy
    fi
    
    python3 wp_generator.py
    
    # Sync to Mission Workspace
    MISSION_DIR="${WS_ROOT}/src/navigation/missions/mission_$w/waypoints"
    mkdir -p "$MISSION_DIR"
    
    for wp_file in outputs/wp*.npy; do
        if [ -f "$wp_file" ]; then
            wp_name=$(basename "$wp_file" .npy)
            mkdir -p "$MISSION_DIR/$wp_name"
            cp "$wp_file" "$MISSION_DIR/$wp_name/$wp_name.npy"
        fi
    done
    
    # Sync costmap, obstacle_data, and aruco_data to mission assets
    ASSETS_DIR="${WS_ROOT}/src/navigation/missions/mission_$w/assets"
    mkdir -p "$ASSETS_DIR"
    if [ -f "$DEV_WORLD/costmap.npz" ]; then
        cp "$DEV_WORLD/costmap.npz" "$ASSETS_DIR/costmap.npz"
    fi
    if [ -f "$DEV_WORLD/obstacle_data.npy" ]; then
        cp "$DEV_WORLD/obstacle_data.npy" "$ASSETS_DIR/obstacle_data.npy"
    fi
    ARUCO_MAP="${WS_ROOT}/src/navstack/slam/roar_localization/config/aruco_map.yaml"
    if [ -f "$ARUCO_MAP" ]; then
        cp "$ARUCO_MAP" "$ASSETS_DIR/aruco_data.yaml"
    fi
done

echo "[3/4] Cleaning Setup Directories..."
rm -f inputs/heightmap.npz inputs/obstacle_data*.npy outputs/wp*.npy

echo "=========================================================="
echo " ✅ SUCCESS! Pipeline Complete!"
echo "=========================================================="
