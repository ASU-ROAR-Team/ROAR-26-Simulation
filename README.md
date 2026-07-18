# ROAR Simulation Workspace (`simulation_ws`)

Welcome to the global ROAR Simulation Workspace. This workspace compiles and manages the complete suite of ROAR simulation packages, including rover models, terrain environments, performance monitors, and navigation setups.

---

## 1. Directory Structure

The `src/` folder is organized into functional categories to group relevant ROS 2 packages together:

*   **`marsyards/`**
    *   **`worlds/`**: Standalone worlds/launch packages containing the customized Gazebo world configurations and the parameterized world launcher.
*   **`rover/`**: ROS 2 packages and configuration assets for the ROAR Rover and arm models.
*   **`monitors/`**: Diagnostic, benchmarking, and telemetry nodes to monitor the simulation in real time.
*   **`navMission_setup/`**: Nodes and launch configurations to set up Nav2 path planning and checkpoints.
*   **`dev_environment/`**: Configuration scripts, container setups, and environment variables.

---

## 2. Getting Started & Building

To compile the entire workspace (including the new `worlds` package):

1.  **Navigate to the workspace root:**
    ```bash
    cd ~/Desktop/ROAR/simulation_ws
    ```
2.  **Sourcing ROS 2 dependencies:**
    Ensure ROS 2 Humble (or your active ROS 2 distribution) is sourced:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
3.  **Compile with Colcon:**
    ```bash
    colcon build --symlink-install
    ```
4.  **Source the Workspace Overlay:**
    ```bash
    source install/setup.bash
    ```

---

## 3. Running Simulation Worlds

With the `worlds` package integrated, you can load and launch any map dynamically:

*   **Launch the Mars Yard world directly:**
    ```bash
    ros2 launch worlds marsyard.launch.py
    ```
*   **Launch a specific world dynamically (using arguments):**
    ```bash
    ros2 launch worlds launch_map.launch.py world:=marsyard.world
    ```

---

## 4. Cleaning Up Between Runs

To avoid node registration collisions or lingering simulator processes, use the global cleanup command before starting a new run:
```bash
RosClean
```
*(This triggers the alias defined in your `.bashrc` to kill all ROS 2, Gazebo, and teleoperation background processes).*
