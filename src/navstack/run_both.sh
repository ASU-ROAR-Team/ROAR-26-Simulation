#!/usr/bin/env bash
# Brings up the full navigation stack: D* Lite global planner + MPPI local controller.
# Usage: ./run_both.sh [map_scenario:=open] [start_x:=0.5] [start_y:=0.5]
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$WS_ROOT/install/setup.bash"
ros2 launch dstar_navigation bringup_full_stack.launch.py "$@"
