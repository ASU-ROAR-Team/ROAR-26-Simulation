# ROAR Simulation Repository — `theOriginal` Branch

Welcome to the `theOriginal` branch of the ROAR Simulation repository. This repository represents the package source space (`src/`) for the ROAR simulation workspace (`simulation_ws`).

---

## 1. Branch Hierarchy & Directory Structure

The repository source space is structured into modular directories representing distinct categories of the simulation:

*   **`dev_environment/`**: Contains Dockerfiles, container setups, and environment customization scripts to configure the workspace dependencies.
*   **`history/`**: Contains archival packages, logs, and development historical versions of simulation assets.
*   **`marsyards/`**: Contains simulation terrain layouts and world models:
    *   **`worlds/`**: The parameterized launcher package containing Gazebo `.world` files (standard Mars Yard footprint) and custom launch scripts.
*   **`monitors/`**: Telemetry, diagnostic, and benchmarking scripts to analyze simulation health and performance.
*   **`navMission_setup/`**: Navigation mission configurations, waypoint lists, and launch setups for Nav2 mapping and navigation.
*   **`panel/`**: Contains the control panel simulation assets, including 3D meshes, URDF/XACRO descriptions, and launch configurations for the interactive panel assembly:
    *   **`src/`**: Modular description packages for all panel components (e.g., switches, rotary knobs, sockets, LEDs, electromagnets, and ArUco markers) and the core `erc_panel_sim` package.
*   **`rover/`**: The core ROS 2 robot description packages (URDF/Xacro, controllers, and meshes) for the ROAR Rover and its robotic arm. It contains three distinct package configurations:
    *   **`roar_rover_clean/`**: Bypasses degradation nodes and arm controllers. Ideal for testing pure navigation and kinematic algorithms.
    *   **`roar_rover_noise/`**: Includes environmental noise (Martian dust, camera glare) and encoder pulse simulations.
    *   **`roar_simulation_full/`**: [Recommended] Combines active environmental degradation, encoder pulse simulation, and the 6-DOF robotic arm.

---

## 2. Onboarding Workflow & Guidelines (For First-Timers)

To ensure smooth collaborative development and maintain a clean git history, all developers working on this branch must adhere to the following rules:

### A. Cloning & Initializing the Repository
*   **Cloning the Branch:**
    If you are connected to a network that blocks SSH (Port 22), clone using the HTTPS remote URL:
    ```bash
    git clone -b theOriginal https://github.com/ASU-ROAR-Team/ROAR-26-Simulation.git
    ```
*   **Initial Setup:**
    Make sure to source the underlying ROS 2 Humble setup file before building:
    ```bash
    source /opt/ros/humble/setup.bash
    ```

### B. Pull Latest Updates Before Working
Before making any changes locally, always fetch and pull the latest modifications from the remote branch to prevent merge conflicts:
```bash
git fetch origin
git pull origin theOriginal
```

### C. Surgical Editing Rule
*   **Only edit files you are actively working on:**
    Do not modify, format, or clean up adjacent files that are unrelated to your assigned task. Keeping your edits surgical makes reviewing diffs straightforward.

### D. Strict Build Protocol
*   **Build from OUTSIDE the `src/` folder:**
    Always navigate to the root of the workspace (`simulation_ws/`) to build the packages. **Never** run `colcon build` from inside the `src/` directory.
    ```bash
    # Navigate to the workspace root
    cd ~/Desktop/ROAR/simulation_ws/
    
    # Compile
    colcon build --symlink-install
    source install/setup.bash
    ```

### E. Committing & Pushing Guidelines (Git Hygiene)
*   **Only push files you need:**
    Avoid running `git add .` blindly. Check modified files using `git status` and stage only the specific source files you changed.
*   **No large binary files:**
    Do not commit zip archives, tarballs, raw 3D mesh files, or database backups to the repository.
*   **Push from INSIDE the repository:**
    Run your commit and push commands from within the cloned repository directory:
    ```bash
    cd ~/Desktop/ROAR/simulation_ws/src/
    git add <specific_file>
    git commit -m "Brief and descriptive commit message"
    git push origin theOriginal
    ```
