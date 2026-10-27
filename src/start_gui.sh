#!/usr/bin/env bash

set -e

SOURCE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROAR_WS="$(cd "${SOURCE_ROOT}/.." && pwd)"
export ROAR_WS

if [ "${1:-}" = "--print-root" ]; then
    echo "${ROAR_WS}"
    exit 0
fi

source /opt/ros/humble/setup.bash

if [ ! -f "${ROAR_WS}/install/setup.bash" ]; then
    echo "[ROAR] Workspace is not built. Building it now..."
    cd "${ROAR_WS}"
    colcon build \
        --symlink-install \
        --executor parallel \
        --allow-overriding nav2_mppi_controller
fi

source "${ROAR_WS}/install/setup.bash"
exec python3 "${SOURCE_ROOT}/navigation/scripts/asu_roar_gui.py"
