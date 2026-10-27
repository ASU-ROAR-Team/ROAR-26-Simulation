# 🖥️ ROAR 2026 — Mission Control GUI & Utility Scripts

[![Module](https://img.shields.io/badge/Module-navigation-E10600?logo=ros&logoColor=white)](file:///home/draaven/ROAR_2026/src/navigation)
[![GUI](https://img.shields.io/badge/GUI-PyQt5_Livery-E10600?logo=qt&logoColor=white)](file:///home/draaven/ROAR_2026/src/navigation/scripts/asu_roar_gui.py)

This directory contains the primary operator interfaces and visualization utilities for the ASU ROAR rover system.

---

## 🎮 Mission Control GUI (`asu_roar_gui.py`)

A full-featured PyQt5 desktop application inspired by Formula 1 telemetry dashboards, designed for competition command and control.

```bash
cd ~/ROAR_2026
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/navigation/scripts/asu_roar_gui.py
```

### Key Modules & Capabilities:
- **01 PREVIEW**: Interactive 2D terrain elevation map with overlaid rock obstacles, waypoint trajectories, and elevation colorbars.
- **02 EVALUATION**: Enable or disable SLAM drift benchmarking, perception evaluation, or pursuit tracking.
- **03 MISSION SETUP**: Select rover degradation model (`clean`, `noise`, `full`), choose target world (`1`, `2`, `3`), and select waypoint ranges.
- **04 LAUNCH**:
  - Automated batch execution: Runs test iterations, handles timeouts, and tears down nodes cleanly between runs.
  - Manual overrides: Single-click buttons to launch Gazebo simulation, Nav2 stack, or referee nodes independently.
- **05 REPORTS**: Real-time iframe rendering of `ERC_Report_Live.html` and aggregated batch summary scorecards (`master_report_batch.html`).

---

## 🗺️ Mission Preview Renderer (`preview_mission.py`)

A standalone CLI utility for generating high-resolution map previews with overlaid waypoints:

```bash
# Preview World 1, Waypoint 0
python3 src/navigation/scripts/preview_mission.py --world 1 --wp 0

# Save render directly to disk without displaying GUI
python3 src/navigation/scripts/preview_mission.py --world 2 --wp 15 --save /tmp/preview_w2_wp15.png
```
