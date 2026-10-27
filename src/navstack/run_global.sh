#!/usr/bin/env bash
# Brings up the D* Lite global planner standalone with a simulated map and path follower.
# Usage: ./run_global.sh [map_scenario:=open] [start_x:=0.5] [start_y:=0.5]
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$WS_ROOT/install/setup.bash"
ros2 launch dstar_navigation bringup_global_planner.launch.py "$@"
