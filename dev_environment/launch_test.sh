#!/bin/bash
# ============================================================
# ROAR Dev Environment - Test Launcher
# Usage: bash launch_test.sh <rover_config> <world_name> [rviz]
#   rover_config: clean | noise | full
#   world_name:   world1 | world2 | world3 | ...
#   rviz:         (optional) add "rviz" to launch RViz
# ============================================================

set -e

ROVER_CONFIG="${1:-clean}"
WORLD_NAME="${2:-world1}"
USE_RVIZ="${3:-}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLDS_DIR="${SCRIPT_DIR}/worlds"

# Map rover config to package name
case "$ROVER_CONFIG" in
    clean) PKG="roar_rover_clean" ;;
    noise) PKG="roar_rover_noise" ;;
    full)  PKG="roar_simulation_full" ;;
    *)
        echo "[ERROR] Unknown rover config: $ROVER_CONFIG"
        echo "        Valid options: clean | noise | full"
        exit 1
        ;;
esac

# Validate world exists
WORLD_DIR="${WORLDS_DIR}/${WORLD_NAME}"
if [ ! -d "$WORLD_DIR" ]; then
    echo "[ERROR] World not found: ${WORLD_DIR}"
    echo "        Available worlds:"
    ls -1 "$WORLDS_DIR" | grep -v README
    exit 1
fi

echo "[launch_sim] Killing any stale Ignition/Gazebo/ROS2 processes..."
pkill -9 -f "ign gazebo"  2>/dev/null || true
pkill -9 -f "ign_gazebo"  2>/dev/null || true
pkill -9 -f "ruby.*ignition" 2>/dev/null || true
pkill -9 -x "rviz2" 2>/dev/null || true
pkill -9 -x "rviz" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "zed_degradation_node" 2>/dev/null || true
pkill -9 -f "encoder_sim_node" 2>/dev/null || true
pkill -9 -f "clean_camera_node" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "teleop" 2>/dev/null || true
pkill -9 -x "gazebo" 2>/dev/null || true
killall -9 ign rviz2 gzserver gzclient ruby gazebo 2>/dev/null || true
sleep 1


WORLD_FILE="${WORLD_DIR}/${WORLD_NAME}.world"
if [ ! -f "$WORLD_FILE" ]; then
    echo "[ERROR] World file not found: ${WORLD_FILE}"
    exit 1
fi

echo "============================================"
echo " ROAR Dev Environment - Test Launch"
echo "============================================"
echo "  Rover Config : ${ROVER_CONFIG} (${PKG})"
echo "  World        : ${WORLD_NAME}"
echo "  World File   : ${WORLD_FILE}"
echo "  RViz         : ${USE_RVIZ:-disabled}"
echo "============================================"

# Kill stale processes
echo "[dev_env] Killing stale Gazebo/ROS2 processes..."
pkill -9 -f "ign gazebo" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

# Clean FastDDS shared memory
echo "[dev_env] Cleaning FastDDS shared memory..."
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true

# Source workspace
if [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
else
    echo "[WARNING] Workspace not built yet. Building now..."
    (cd "${SCRIPT_DIR}" && colcon build --parallel-workers 1 --executor sequential --paths rover/*)
    source "${SCRIPT_DIR}/install/setup.bash"
fi

echo "[dev_env] Launching simulation..."

# Set environment
export QT_QPA_PLATFORM=xcb
MODELS_DIR="${SCRIPT_DIR}/models"
ERC_MODELS_DIR="${SCRIPT_DIR}/../marsyards/marsyard/models"
ROCKS_DIR="${SCRIPT_DIR}/../navMission_setup/world_setup/rocks_ws"
SIM_RESOURCE_PATH="${ERC_MODELS_DIR}:${ROCKS_DIR}:${MODELS_DIR}:${SCRIPT_DIR}/install"
export IGN_GAZEBO_RESOURCE_PATH="${SIM_RESOURCE_PATH}:${IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${SIM_RESOURCE_PATH}:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib"

echo "[dev_env] Resource path: ${IGN_GAZEBO_RESOURCE_PATH}"

SPAWN_X="${4:-0.0}"
SPAWN_Y="${5:-0.0}"

# Launch — pass world file and initial spawn pose to launch file
if [ "$USE_RVIZ" = "rviz" ]; then
    ros2 launch "$PKG" basic_rover.launch.py world_file:="$WORLD_FILE" start_rviz:=true spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y"
else
    ros2 launch "$PKG" basic_rover.launch.py world_file:="$WORLD_FILE" spawn_x:="$SPAWN_X" spawn_y:="$SPAWN_Y"
fi
