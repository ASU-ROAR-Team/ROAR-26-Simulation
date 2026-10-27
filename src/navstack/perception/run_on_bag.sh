#!/bin/bash
# ============================================================
#  Run ROAR Perception Pipeline on ZED2i ROS2 Bag
#
#  Usage:
#    bash ~/new_roar_repo/nav-stack_2026/perception/run_on_bag.sh [path/to/bag] [rate]
#
#  Defaults:
#    bag  → most-recent bag in ~/zed2i_bags/
#    rate → 0.5
#
#  What it does (in order):
#    1. Sources ROS2 + the perception_module workspace (nav-stack_2026)
#    2. Launches RViz2 with the perception pipeline config
#    3. Launches the perception stack (lidar_cluster, aruco,
#       obstacle_publisher, bev_visualizer) with use_sim_time=true
#    4. Plays back the bag with --clock so all nodes use
#       bag timestamps instead of wall clock
# ============================================================

# ── Resolve workspace root relative to this script's location ────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PERCEPTION_WS="$(cd "$SCRIPT_DIR/.." && pwd)"   # nav-stack_2026/

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; NC='\033[0m'

RVIZ_CONFIG="$HOME/perception_pipeline.rviz"

echo -e "${BOLD}${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║    ROAR Perception Pipeline — ZED2i Bag Playback + RViz     ║"
echo "╚══════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "   ${BOLD}Workspace:${NC} $PERCEPTION_WS"
echo ""

# ── Source ROS2 ──────────────────────────────────────────────────────────
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
  echo -e "${GREEN}✓ ROS2 Humble sourced${NC}"
else
  echo -e "${RED}[ERROR] ROS2 Humble not found at /opt/ros/humble/setup.bash${NC}"; exit 1
fi

# ── Source perception workspace ──────────────────────────────────────────
if [ -f "$PERCEPTION_WS/install/setup.bash" ]; then
  source "$PERCEPTION_WS/install/setup.bash"
  echo -e "${GREEN}✓ perception_module workspace sourced${NC}"
else
  echo -e "${RED}[ERROR] Workspace not built yet. Run:${NC}"
  echo -e "   cd $PERCEPTION_WS"
  echo -e "   colcon build --packages-select perception_module --symlink-install"
  exit 1
fi

# ── Resolve bag path ─────────────────────────────────────────────────────
if [ -n "$1" ]; then
  BAG_PATH="$1"
else
  BAG_PATH=$(ls -dt ~/zed2i_bags/zed2i_*/ 2>/dev/null | head -1)
  if [ -z "$BAG_PATH" ]; then
    echo -e "${RED}[ERROR] No bag found in ~/zed2i_bags/ — pass a path as argument${NC}"
    echo -e "   Usage: bash run_on_bag.sh /path/to/bag [rate]"
    exit 1
  fi
  echo -e "${YELLOW}No bag specified — using most recent:${NC}"
fi
echo -e "   ${BOLD}Bag :${NC} $BAG_PATH"

if [ ! -d "$BAG_PATH" ]; then
  echo -e "${RED}[ERROR] Bag directory not found: $BAG_PATH${NC}"; exit 1
fi

# ── Resolve playback rate ─────────────────────────────────────────────────
RATE="${2:-0.5}"
echo -e "   ${BOLD}Rate:${NC} ${RATE}x"

# ── Show bag info ─────────────────────────────────────────────────────────
echo ""
echo -e "${CYAN}Bag info:${NC}"
ros2 bag info "$BAG_PATH" 2>/dev/null | grep -E "Duration|Size|Messages|Topic"
echo ""

# ── Cleanup on exit ───────────────────────────────────────────────────────
PIDS=()
cleanup() {
  echo ""
  echo -e "${YELLOW}Shutting down all processes...${NC}"
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null
  done
  wait 2>/dev/null
  echo -e "${GREEN}Done.${NC}"
}
trap cleanup EXIT INT TERM

# ── [1] Launch RViz2 ─────────────────────────────────────────────────────
echo -e "${CYAN}[1/3] Launching RViz2...${NC}"
if [ -f "$RVIZ_CONFIG" ]; then
  rviz2 -d "$RVIZ_CONFIG" &
  echo -e "${GREEN}   ✓ RViz2 started with config: $RVIZ_CONFIG${NC}"
else
  rviz2 &
  echo -e "${YELLOW}   ⚠ No config found at $RVIZ_CONFIG — opening blank RViz2${NC}"
fi
RVIZ_PID=$!
PIDS+=($RVIZ_PID)
sleep 2

# ── [2] Launch perception stack ──────────────────────────────────────────
echo ""
echo -e "${CYAN}[2/3] Launching perception stack (use_sim_time=true)...${NC}"

PERCEPTION_LOG="/tmp/perception_launch.log"
ros2 launch perception_module perception.launch.py \
  use_sim_time:=true \
  > "$PERCEPTION_LOG" 2>&1 &
PERCEPTION_PID=$!
PIDS+=($PERCEPTION_PID)
echo -e "${GREEN}   ✓ Perception stack PID: $PERCEPTION_PID${NC}"
echo -e "   Log: tail -f $PERCEPTION_LOG"

# Give perception nodes time to fully initialise before bag starts sending data
echo -e "${YELLOW}   Waiting 6 s for nodes to initialise...${NC}"
sleep 6

# Verify perception stack is still running after startup
if ! kill -0 "$PERCEPTION_PID" 2>/dev/null; then
  echo -e "${RED}[ERROR] Perception stack crashed during startup!${NC}"
  echo -e "   Check log: cat $PERCEPTION_LOG"
  exit 1
fi
echo -e "${GREEN}   ✓ Perception stack running OK${NC}"

# ── [3] Play bag with sim clock ───────────────────────────────────────────
echo ""
echo -e "${CYAN}[3/3] Starting bag playback at ${RATE}x...${NC}"
echo -e "${BOLD}Press Ctrl+C to stop everything.${NC}"
echo ""

ros2 bag play "$BAG_PATH" \
  --clock \
  --rate "$RATE" \
  --read-ahead-queue-size 10

# bag play exiting triggers the trap → cleanup
