#!/usr/bin/env bash
# =============================================================================
# roar_sim.sh - Clean launch script for the ROAR rover simulation
# Usage:  bash roar_sim.sh [rviz]
# =============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 1. Source the workspace ───────────────────────────────────────────────────
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo "[roar_sim] ERROR: install/setup.bash not found."
    echo "           Run 'colcon build' from $WORKSPACE_DIR first."
    exit 1
fi
source "$WORKSPACE_DIR/install/setup.bash"

# ── 2. Kill stale Gazebo / ign processes and ROS2 nodes ───────────────────────
# Without this, the second launch hits amdgpu_query_info(ACCEL_WORKING)=-13
# (EACCES) because the GPU DRM device is still held by the previous session.
echo "[roar_sim] Killing any stale Ignition/Gazebo/ROS2 processes..."
pkill -9 -f "ign gazebo"  2>/dev/null || true
pkill -9 -f "ign_gazebo"  2>/dev/null || true
pkill -9 -f "ruby.*ignition" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
pkill -9 -f "robot_state_publisher" 2>/dev/null || true
pkill -9 -f "spawner" 2>/dev/null || true
pkill -9 -f "ros2" 2>/dev/null || true
pkill -9 -f "zed_degradation_node" 2>/dev/null || true
pkill -9 -f "encoder_sim_node" 2>/dev/null || true
pkill -9 -f "clean_joint_state_publisher" 2>/dev/null || true
sleep 1   # give the kernel time to release GPU + DRM file descriptors

# ── 3. Clean stale FastDDS shared-memory ports ───────────────────────────────
echo "[roar_sim] Cleaning stale FastDDS shared memory..."
rm -f /dev/shm/fastrtps_port* 2>/dev/null || true
rm -f /tmp/.ign_gazebo_*      2>/dev/null || true

# ── 4. Set environment BEFORE any child process is forked ────────────────────
# QT_QPA_PLATFORM=xcb → force X11 (Xwayland) so Qt/Ogre don't fight over
#                        native Wayland EGL contexts.
export QT_QPA_PLATFORM=xcb

# Resource paths so Ignition resolves model:// URIs to package meshes.
INSTALL_DIR="$WORKSPACE_DIR/install"
export IGN_GAZEBO_RESOURCE_PATH="${IGN_GAZEBO_RESOURCE_PATH:+$IGN_GAZEBO_RESOURCE_PATH:}$INSTALL_DIR"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$INSTALL_DIR"

# Plugin search path — Ignition uses this to find system plugins like ign_ros2_control-system.
# WITHOUT this, the plugin loader cannot find libign_ros2_control-system.so even though
# /opt/ros/humble/lib is in LD_LIBRARY_PATH (Ignition's plugin loader bypasses ldconfig).
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="/opt/ros/humble/lib:${IGN_GAZEBO_SYSTEM_PLUGIN_PATH}"


# ── 4. Launch ─────────────────────────────────────────────────────────────────
LAUNCH_FILE="basic_rover.launch.py"

# Check if the first argument specifies which launch file to use
if [[ "$1" == "clean" ]]; then
    LAUNCH_FILE="basic_rover_clean.launch.py"
    shift
elif [[ "$1" == "basic" ]]; then
    LAUNCH_FILE="basic_rover.launch.py"
    shift
elif [[ "$1" == *"launch.py" ]]; then
    LAUNCH_FILE="$1"
    shift
fi

START_RVIZ="false"
if [[ "$1" == "rviz" ]]; then
    START_RVIZ="true"
    shift
fi

echo "[roar_sim] Launching ROAR rover simulation..."
echo "           LAUNCH_FILE                  = $LAUNCH_FILE"
echo "           QT_QPA_PLATFORM              = $QT_QPA_PLATFORM"
echo "           LIBGL_ALWAYS_SOFTWARE        = $LIBGL_ALWAYS_SOFTWARE"
echo "           IGN_GAZEBO_RESOURCE_PATH     = $IGN_GAZEBO_RESOURCE_PATH"
echo "           IGN_GAZEBO_SYSTEM_PLUGIN_PATH= $IGN_GAZEBO_SYSTEM_PLUGIN_PATH"
echo ""

exec ros2 launch roar_simulation "$LAUNCH_FILE" \
    start_rviz:=$START_RVIZ \
    "$@"
