# ERC Full Simulation Performance Report

## Simulation Parameters
- Map Size: 400x400 cells (20.0m x 20.0m)
- Start Pose: (1.0, 1.0)
- Goal Pose: (19.0, 19.0)
- Travel Duration: 90.55 seconds
- Replans Executed: 2

## Key Findings
- **Scale**: The planner operated on a 160,000-cell grid without any latency spikes.
- **Replanning**: Dynamic canyon obstacles were successfully injected, triggering rapid incremental updates.
- **Correctness**: Robot successfully bypassed canyons and rough gravel zones to reach the goal.
