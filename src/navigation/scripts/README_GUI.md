# ASU ROAR Mission Control GUI

**Location:** `Navigation_Mission_Workspace/src/navigation/scripts/asu_roar_gui.py`

This GUI is the central hub for the Simulation & Testing Team to orchestrate and evaluate the Navigation pipeline. It provides a full PyQt5 interface for loading waypoints, queueing test runs, and generating Master Evaluation Reports.

## Features & Workflow

### 1. Mission Setup: Single Waypoint vs. Batch Mode

The GUI supports two main ways of running navigation missions:

#### **A. Single Waypoint Mode (Manual Testing)**
Use this mode when you want to quickly test a specific waypoint in a specific world without setting up a full pipeline sequence.
1. **Select World**: Pick the Marsyard world you want to run (e.g., World 1).
2. **Select Waypoint**: Set the exact Waypoint ID you want the rover to navigate to. 
3. The GUI will load the corresponding `costmap.npz` and fetch the exact `x, y` real-world coordinates for that waypoint.
4. When you start the simulation, the system will execute just this single objective.

#### **B. Batch Mode (Automated Pipeline)**
Use this mode for unattended, large-scale testing (e.g., running overnight).
1. **Define Range**: Use the *FROM WORLD*, *TO WORLD* and *FROM WP*, *TO WP* spinboxes to define a range of tests.
2. **Add to Queue**: Click **ADD TO BATCH QUEUE**. You can stack multiple different world and waypoint combinations into the queue list on the right side of the GUI.
3. **Automated Execution**: The GUI will seamlessly transition between them without requiring any manual interaction. It automatically injects the `batch_waypoint_follower.py` script to navigate the rover, kills all processes when a mission finishes, generates the report, and starts the next item in the queue.

### 2. Orchestration & Launch
In the **LAUNCH** tab, you control how the simulation executes:
- **Rover Config:** Choose between `clean`, `noise`, or `full` rover modules depending on what sub-systems you are testing. 
- **Headless Mode:** By default, it runs Gazebo in headless mode to maximize hardware resources and speed up testing. You can also easily toggle RViz2 on or off for visual monitoring.
- **Run Pipeline:** When you click **RUN BATCH PIPELINE**, the GUI dynamically:
  1. Updates your navigation stack's `roar_nav2_sim.yaml` with the current mission's specific waypoints.
  2. Launches the `master_evaluation.launch.py` environment.
  3. Waits 30 seconds for the system to stabilize.
  4. Automatically navigates the rover and manages the lifecycle of the test.

### 3. Evaluation & Reporting
In the **EVALUATION** and **REPORTS** tabs:
- You can enable or disable specific evaluation suites (SLAM Benchmarking, Perception Testing, Pure Pursuit).
- Once an item finishes (single or batch), the GUI aggregates the Referee Engine results and SLAM metrics, parsing them into a highly readable, consolidated **Master Evaluation Report** (HTML).
- You can view these comprehensive HTML reports right from the interface by clicking the report buttons.

## How to Run
Ensure both your Simulation and Navigation workspaces are built and sourced. Make sure the `ROAR_SIM_WS` environment variable is set or the simulation is at the default path (`${ROAR_WS}/src/roar2026`).

```bash
cd ${ROAR_WS}
source install/setup.bash
python3 src/navigation/scripts/asu_roar_gui.py
```
