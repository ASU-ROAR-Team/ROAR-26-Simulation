# ROAR-26 Simulation & Navigation Pipeline

This document illustrates the complete end-to-end pipeline for generating procedural Marsyard simulation worlds, testing them in the development environment, and syncing them to the final Navigation Evaluation Workspace.

## 🏗️ Architecture & Pipeline Flow
```mermaid
graph TD
    %% Generation Pipeline
    subgraph "navMission_setup (Generation)"
        A[build_full_worlds.sh] -->|Generates Rocks| B(obsData_gen)
        B -->|Flags Meshes 5,6,7,8,9 as Safe| C[obstacle_data.npy]
        B -->|Places 15 ArUcos| D(world_gen)
        C --> D
        D -->|Fuses XML| E[marsyard.world]
    end

    %% Testing Sandbox
    subgraph "dev_environment (Testing)"
        E -->|Copies to| F[dev_environment/worlds/worldX]
        C -->|Copies to| F
        F -->|Manual Verification| G[bash launch_test.sh]
    end

    %% Syncing & Mission Setup
    subgraph "Navigation_Mission_Workspace (Evaluation)"
        H[master_pipeline.sh] -.->|Scans| F
        H -->|Calculates Paths| I(wp_generator.py)
        I -->|Outputs| J[wp01.npy, wp02.npy, etc.]
        F -->|Syncs Worlds & Data| K[missions/mission_X/]
        J -->|Syncs Waypoints| K
        
        K -->|Loads Data| L[Referee Engine]
        L -->|Reads obstacle_data.npy| M{Is Collision Safe?}
        M -->|Yes (Mesh 5,6,7,8,9)| N[Ignore Contact]
        M -->|No (Mesh 1,3)| O[Trigger Fatal Alarm!]
    end
```

## 1. World Generation (`navMission_setup`)
The core world generation logic lives inside `navMission_setup/`. 

To generate completely new randomized rock layouts and ArUco markers, run:
```bash
cd navMission_setup
bash build_full_worlds.sh
```
**What this does:**
* Calls the `obsData_gen` script to generate randomized rock placements (8 rocks for World 1, 50 for World 2, 90 for World 3).
* Measures the physical height of each generated rock. If a rock uses specific flat meshes (e.g., Meshes 5, 6, 7, 8, 9), it automatically flags them as **Non-Collidable** (`is_collidable = False`) so they don't trigger fatal failure alarms in the Referee Engine.
* Injects all the rocks and 15 randomized ArUco markers into the raw Gazebo XML (`marsyard.world`).
* Automatically copies these fully formed worlds into the Testing Workspace (`dev_environment/worlds/worldX/`).

## 2. The Development Testing Environment (`dev_environment`)
Once the worlds are built, they live in `dev_environment`. This is your sandbox for visually testing the generated worlds, making sure the rover can spawn correctly, and verifying that the physics and sensors (like the ZED camera or lidar) are working as expected.

*(See `USE_ME.md` for specific instructions on how to launch the rover in this environment).*

## 3. Mission Syncing (`master_pipeline.sh`)
When you are satisfied with the worlds generated in the dev environment, you need to package them up and sync them over to the headless evaluation engine.

Run the master pipeline script:
```bash
cd navMission_setup
bash master_pipeline.sh
```
**What this does:**
* Scans the `dev_environment/worlds` to find the newest `obstacle_data.npy` files.
* Passes the rock coordinate data to the Waypoint Generator (`wp_generator.py`), which plots out a safe, collision-free waypoint path through the Marsyard.
* Copies the final `marsyard.world`, the waypoint paths, the `obstacle_data.npy`, and the `aruco_data.yaml` directly into your **Navigation Mission Workspace** (`Navigation_Mission_Workspace/missions/mission_X/`).

## 4. Headless Evaluation (Referee Engine)
Once `master_pipeline.sh` finishes, the worlds are fully locked in and ready for autonomous evaluation. 

When you launch the GUI (`asu_roar_gui.py`) and start a Batch Test:
1. The Referee Engine loads the `obstacle_data.npy` file.
2. The Waypoint Follower attempts to navigate the pre-generated path.
3. If the rover steps on a flat rock (like Mesh 5), the Referee Engine checks its internal safe-list and silently ignores the contact.
4. If the rover hits a dangerous rock (like Mesh 1 or 3), the Referee Engine instantly throws a `True` collision flag, failing the iteration.
