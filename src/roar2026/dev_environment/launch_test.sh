#!/bin/bash
# ============================================================
# ROAR Dev Environment - Test Launcher
# Usage:
#   bash launch_test.sh <rover_config> <world_name> [options]
#   rover_config: clean | noise | full
#   world_name:   world1 | world2 | world3 | ...
#
# Options (flags or positional):
#   --rviz          (or positional arg 3: "rviz") Launch RViz
#   --headless      (or positional arg 3: "headless") Run Gazebo without GUI
#   --x <val>       (or positional arg 4) Initial spawn X (default: 0.0)
#   --y <val>       (or positional arg 5) Initial spawn Y (default: 0.0)
#   --z <val>       (or positional arg 6) Initial spawn Z (default: 0.5)
# ============================================================

set -e

ROVER_CONFIG="${1:-clean}"
WORLD_NAME="${2:-world1}"
shift $(( $# >= 2 ? 2 : $# ))

USE_RVIZ="false"
HEADLESS="false"
SPAWN_X="0.0"
SPAWN_Y="0.0"
SPAWN_Z="0.5"
POS_IDX=0

while [ $# -gt 0 ]; do
    case "$1" in
        --rviz)     USE_RVIZ="true"; shift ;;
        --headless) HEADLESS="true"; shift ;;
        --x)        SPAWN_X="${2:?--x requires a value}"; shift 2 ;;
        --y)        SPAWN_Y="${2:?--y requires a value}"; shift 2 ;;
        --z)        SPAWN_Z="${2:?--z requires a value}"; shift 2 ;;
        -h|--help)
            sed -n '2,15p' "$0"
            exit 0
            ;;
        *)
            POS_IDX=$((POS_IDX + 1))
            case "$POS_IDX" in
                1)
                    if [ "$1" = "rviz" ]; then
                        USE_RVIZ="true"
                    elif [ "$1" = "headless" ]; then
                        HEADLESS="true"
                    fi
                    ;;
                2) SPAWN_X="$1" ;;
                3) SPAWN_Y="$1" ;;
                4) SPAWN_Z="$1" ;;
            esac
            shift
            ;;
    esac
done

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
echo "  Spawn Pose   : (${SPAWN_X}, ${SPAWN_Y}, ${SPAWN_Z})"
echo "  RViz         : ${USE_RVIZ}"
echo "  Headless     : ${HEADLESS}"
echo "============================================"

# Targeted cleanup of stale simulation processes (avoids killing user teleop/CLI terminals)
echo "[dev_env] Cleaning stale Gazebo/simulation processes..."
pkill -9 -f "ign gazebo"            2>/dev/null || true
pkill -9 -f "ign_gazebo"            2>/dev/null || true
pkill -9 -f "gz sim"                2>/dev/null || true
pkill -9 -f "ruby.*ignition"        2>/dev/null || true
pkill -9 -x "rviz2"                 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "controller_manager/spawner" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge"         2>/dev/null || true
pkill -9 -f "ros_gz_sim"            2>/dev/null || true
pkill -9 -f "zed_degradation_node"  2>/dev/null || true
pkill -9 -f "encoder_sim_node"      2>/dev/null || true
pkill -9 -f "clean_camera_node"     2>/dev/null || true
pkill -9 -f "collision_alarm_node"  2>/dev/null || true
pkill -9 -f "cmd_vel_relay"         2>/dev/null || true
sleep 1

# Clean FastDDS shared memory
echo "[dev_env] Cleaning FastDDS shared memory..."
rm -rf /dev/shm/fastrtps_* 2>/dev/null || true

# Source workspace
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    source "${WS_ROOT}/install/setup.bash"
elif [ -f "${SCRIPT_DIR}/install/setup.bash" ]; then
    source "${SCRIPT_DIR}/install/setup.bash"
else
    echo "[dev_env] Sourcing ROS 2 Humble base..."
    source /opt/ros/humble/setup.bash
fi

echo "[dev_env] Launching simulation..."

# Set environment
export QT_QPA_PLATFORM=xcb
ERC_MODELS_DIR="${SCRIPT_DIR}/../marsyards/marsyard/models"
ROCKS_DIR="${SCRIPT_DIR}/../navMission_setup/world_setup/rocks_ws"
SIM_RESOURCE_PATH="${ERC_MODELS_DIR}:${ROCKS_DIR}:${WS_ROOT}/install:${SCRIPT_DIR}/install"
export IGN_GAZEBO_RESOURCE_PATH="${SIM_RESOURCE_PATH}:${IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="${SIM_RESOURCE_PATH}:${GZ_SIM_RESOURCE_PATH}"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib"

echo "[dev_env] Resource path: ${IGN_GAZEBO_RESOURCE_PATH}"

exec ros2 launch "$PKG" basic_rover.launch.py \
    world_file:="$WORLD_FILE" \
    start_rviz:="$USE_RVIZ" \
    headless:="$HEADLESS" \
    spawn_x:="$SPAWN_X" \
    spawn_y:="$SPAWN_Y" \
    spawn_z:="$SPAWN_Z"
