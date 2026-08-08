#!/usr/bin/env bash
# run_pipeline.sh — Full offline planning pipeline in one command.
# Reads all parameters from config.yaml.
# Run from the offline_tools/ directory: ./run_pipeline.sh
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Parse config.yaml with python (no extra deps beyond pyyaml)
cfg() { python3 -c "import yaml; c=yaml.safe_load(open('config.yaml')); print($1)"; }

RESOLUTION=$(cfg "c['heightmap']['resolution']")
SWAP_XY=$(cfg "'--swap-xy' if c['heightmap']['swap_xy'] else ''")
PREFER=$(cfg "c['heightmap']['prefer']")
YAW=$(cfg "c['heightmap']['yaw']")
MAX_SLOPE=$(cfg "c['costmap']['max_slope']")
MAX_HDIFF=$(cfg "c['costmap']['max_height_diff']")
SMOOTH=$(cfg "c['costmap']['smooth_sigma']")
MESH=$(cfg "c['paths']['mesh_stl']")
HEIGHTMAP=$(cfg "c['paths']['heightmap_npz']")
COSTMAP=$(cfg "c['paths']['costmap_csv']")

echo "=== Step 1/3: Rasterise terrain mesh → heightmap ==="
python3 scripts/mapping/generate_heightmap.py \
    "$MESH" \
    --output "$HEIGHTMAP" \
    --resolution "$RESOLUTION" \
    --prefer "$PREFER" \
    --yaw "$YAW" \
    $SWAP_XY

echo ""
echo "=== Step 2/3: Convert heightmap → traversability costmap ==="
python3 scripts/mapping/convert_heightmap_to_costmap.py \
    --input "$HEIGHTMAP" \
    --output "$COSTMAP" \
    --resolution "$RESOLUTION" \
    --max-slope "$MAX_SLOPE" \
    --max-height-diff "$MAX_HDIFF" \
    --smooth "$SMOOTH"

echo ""
echo "=== Step 3/3: Plan sequence with D* (ROS 2 launch) ==="
echo "    (close RViz when you are satisfied to end this step)"
source /home/amrtamer/nav-stack_2026/install/setup.bash
ros2 launch dstar_navigation offline_planner.launch.py

echo ""
echo "Done.  Outputs written to data/"
