#!/usr/bin/env bash
set -e

ROOT="$HOME/new_sim_tes/ROAR-26-Simulation/navMission_setup"
WORLD="$ROOT/outputs/world2/world2.world"
MODELS="$ROOT/outputs/world2/models"

export GZ_SIM_RESOURCE_PATH="$MODELS:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$MODELS:${IGN_GAZEBO_RESOURCE_PATH:-}"

echo "======================================"
echo "Launching latest WORLD 2"
echo "World: $WORLD"
echo "Models: $MODELS"
echo "======================================"

ign gazebo "$WORLD"
