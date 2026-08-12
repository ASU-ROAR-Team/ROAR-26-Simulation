#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "=========================================================="
echo "    MARS YARD WORLD & ARUCO MARKER GENERATION PIPELINE"
echo "=========================================================="

export IGN_GAZEBO_RESOURCE_PATH="${SCRIPT_DIR}/world_setup/rocks_ws"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/world_setup/rocks_ws"

echo -e "\n[1/3] Generating Core Worlds (world_1, world_2, world_3)..."
cd "${SCRIPT_DIR}"
rm -rf outputs/world_1 outputs/world_2 outputs/world_3
python3 generate_worlds.py

echo -e "\n[2/3] Syncing Outputs to Development Environment..."
SRC_DIR="${SCRIPT_DIR}/outputs"
DEST_DIR="${WORKSPACE_DIR}/dev_environment/worlds"
for i in 1 2 3; do
    SRC="${SRC_DIR}/world_${i}"
    DEST="${DEST_DIR}/world${i}"
    
    if [ -f "${DEST}/world${i}.launch.py" ]; then
        cp "${DEST}/world${i}.launch.py" "/tmp/world${i}.launch.py.bak" || true
    fi
    
    rm -rf "${DEST}"
    mkdir -p "${DEST}"
    
    cp "${SRC}/world/"*.world "${DEST}/world${i}.world"
    cp "${SRC}/heightmap/"*.npz "${DEST}/heightmap.npz"
    cp "${SRC}/heightmap/"*.png "${DEST}/heightmap.png"
    cp "${SRC}/costmap/"*.npz "${DEST}/costmap.npz"
    cp "${SRC}/costmap/"*.png "${DEST}/costmap.png"
    cp -r "${SRC}/costmap/csv" "${DEST}/csv" 2>/dev/null || true
    cp "${SRC}/obstacle_data/obstacle_data.npy" "${DEST}/obstacle_data.npy"
    
    if [ -f "${SRC}/obstacle_data/obstacle_data_info.txt" ]; then
        cp "${SRC}/obstacle_data/obstacle_data_info.txt" "${DEST}/obstacle_data_info.txt"
    fi
    if [ -f "${SRC}/metadata.txt" ]; then
        cp "${SRC}/metadata.txt" "${DEST}/metadata.txt"
    fi
    if [ -f "/tmp/world${i}.launch.py.bak" ]; then
        mv "/tmp/world${i}.launch.py.bak" "${DEST}/world${i}.launch.py"
    fi
done

echo -e "\n[3/3] Fusing ArUco Markers into Generated Worlds..."
cd "${WORKSPACE_DIR}"
for i in 1 2 3; do
    echo "  -> Injecting into world_$i"
    HEIGHTMAP=$(ls "${SCRIPT_DIR}/outputs/world_${i}/heightmap/"*.npz | head -n 1)
    BASE_WORLD="${DEST_DIR}/world${i}/world${i}.world"
    
    python3 "${SCRIPT_DIR}/world_setup/TempArucoGen/scripts/step2_generate_npy.py" \
        --heightmap "$HEIGHTMAP" \
        --output-dir "/tmp/aruco_data_${i}"
        
    python3 "${SCRIPT_DIR}/world_setup/TempArucoGen/scripts/step3_fuse_world.py" \
        --base-world "$BASE_WORLD" \
        --output-world "$BASE_WORLD" \
        --npy-data "/tmp/aruco_data_${i}/aruco_data.npy"
done

# Ensure ArUco models exist in the gazebo workspace
cp -r "${SCRIPT_DIR}/world_setup/TempArucoGen/models/aruco_"* "${WORKSPACE_DIR}/marsyards/marsyard/models/" 2>/dev/null || true

echo -e "\n=========================================================="
echo " ✅ SUCCESS! All worlds generated and correctly integrated!"
echo "    Test with: ./launch_test.sh full world1 rviz"
echo "=========================================================="
