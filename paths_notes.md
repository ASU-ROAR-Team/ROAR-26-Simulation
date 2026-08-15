# Critical Directory Naming Instructions

To ensure the automated testing pipelines, world generation scripts, and Mission Control GUI function properly, all team members **MUST** clone the repositories into their home directory (`~/`) using these exact folder names:

1. **Simulation Repository**: `~/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion`
   *(Note the exact spelling of 'Simualtion' is required by the shell scripts).*
2. **Navigation Mission Workspace**: `~/Navigation_Mission_Workspace`
3. **Navigation Stack 2026**: `~/navStack_2026_Workspace`

If you clone them with different names (e.g., omitting the `_Workspace` suffix), the `master_pipeline.sh` orchestrator and the Python pathing logic will fail to locate the necessary assets, generated worlds, and waypoints!
