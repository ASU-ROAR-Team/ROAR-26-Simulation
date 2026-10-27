#!/usr/bin/env bash
# Brings up the MPPI local controller standalone. Subscribes to /global_plan.
# Usage: ./run_local.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
source "$WS_ROOT/install/setup.bash"
ros2 launch dstar_navigation bringup_local_controller.launch.py "$@"
