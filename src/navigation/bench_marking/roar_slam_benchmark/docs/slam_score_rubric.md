# SLAM Score Rubric

The SLAM Evaluator generates a 0-100 score quantifying the state-estimation fidelity of a simulation run. This rubric is used by judges to evaluate the localization team's output.

## Scoring Components

The final SLAM score aggregates four distinct metrics:

1. **Trajectory Following (35% Weight)**
   * **What it measures:** The 2D Root Mean Square Error (RMSE) between the estimated position (`/odom`) and ground truth (`/ground_truth/pose`).
   * **Why it matters:** Trajectory deviation is the primary indicator of how well the SLAM filter tracks the rover's actual movement.

2. **Drift Rate (15% Weight)**
   * **What it measures:** The localized trajectory error increase over time, specifically the maximum trajectory drift rate over any 30-second window.
   * **Why it matters:** Brief periods of high error (e.g. lost ArUco tracking) can ruin an entire mission. This penalizes sudden jumps or localized tracking losses that global averages might hide.

3. **Static Drift / Phantom Movement (30% Weight)**
   * **What it measures:** The distance the SLAM filter claims the rover has moved when the commanded velocity (`/cmd_vel`) has been strictly zero for an extended period.
   * **Why it matters:** This strictly identifies state-estimation defects (e.g. ZUPT not firing, unchecked IMU noise) independent of the controller's driving accuracy. A rover that thinks it's moving when parked is fundamentally dangerous.

4. **Map Topography Validation (20% Weight)**
   * **What it measures:** The F1-score of correctly identifying lethal obstacles in the generated costmap compared to the true heightmap, along with elevation RMSE (when available).
   * **Why it matters:** SLAM isn't just about trajectory; the resulting generated maps must be geometrically accurate and trustworthy for path planning.

## Static Drift Hard-Fail Rule

> **Important:** Phantom drift is penalized with a "Hard-Fail" cap.

If the rover's estimated pose drifts at a rate exceeding **0.10 m/s** while parked (commanded 0 velocity), the final SLAM score will be hard-capped at a maximum of **25/100**, regardless of how excellent the other metrics are. 

**Justification:** A rover that reports significant movement while stationary cannot be trusted. The filter's covariance or motion model is broken, representing a severe and potentially catastrophic system failure.

## Worked Example

Below is a breakdown of how the scoring algorithm evaluates a sample run:

| Metric | Raw Value | Reference Value (37% Score) | Normalized Score (0-1) | Weight | Final Points |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Trajectory** | 0.2 m RMSE | 0.5 m | 0.670 | 0.35 | 23.46 |
| **Drift Rate** | 0.02 m/s | 0.05 m/s | 0.670 | 0.15 | 10.05 |
| **Static Drift** | 0.005 m/s | 0.01 m/s | 0.606 | 0.30 | 18.19 |
| **Topography** | F1: 0.8 | N/A | 0.800 (F1) | 0.20 | 16.00 |
| **Total** | | | | **1.00** | **67.70 / 100** |

*In this example, the static drift was below the 0.10 m/s hard-fail threshold, so the score remains 67.70.*

## Non-Goals
This rubric does **NOT** score:
- Driving skill or controller accuracy (e.g. staying on an exact path).
- Path planning efficiency (e.g. finding the shortest route).
- Obstacle avoidance behavior. 
It strictly evaluates state-estimation fidelity.
