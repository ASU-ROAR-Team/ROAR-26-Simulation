# D* Lite Unified Costmap Refactor — Changelog & Tuning Guide

## Summary of Changes

The D* Lite global planner has been refactored to consume a single `/unified_costmap` generated directly from the 2.5D heightmap published by SLAM (`/active_map/heightmap`). The 2D `/map` subscription has been completely removed as it is redundant.

---

## Architecture: Pure Heightmap Pipeline

```
/active_map/heightmap           ──┐
/active_map/heightmap_range     ──┴──→  heightmap_to_costmap.py  ──→  /unified_costmap  ──→  D* Planner  ──→  /global_plan
                                        (Derivatives + Inflation)       (100 = lethal)       - clearance_field_term
                                                                        (0-99 = terrain)     - terrain_term
```

---

## Detailed Changes

### 1. `heightmap_to_costmap.py` (Heightmap-Only + Inflation)

- Subscribes to `/active_map/heightmap` and `/active_map/heightmap_range` (No `/map` subscription).
- Computes **1st derivative** (steepness via Sobel) and **2nd derivative** (roughness via Laplacian).
- Hard-blocks steep slopes ($> \text{MAX\_SAFE\_SLOPE}$) and height steps ($> \text{max\_traversable\_height\_diff}$) as lethal (100).
- Applies a **2-stage OpenCV distance-transform inflation layer** around all height-derived lethal cells:
  1. `inscribed_radius` (default 0.25m): Cells within this radius of any lethal cell become **hard lethal (100)** to account for the robot's physical footprint.
  2. `inflation_radius` (default 0.8m): Cells in the band between `inscribed_radius` and `inflation_radius` get exponentially decaying inflation costs ($1 - 99$).
- Non-lethal cell cost is $\max(\text{terrain\_cost}, \text{inflation\_cost})$.

---

## Parameter Tuning Guide

### Costmap & Inflation Parameters (`heightmap_to_costmap`)

| Parameter | Default | Effect | How to Tune |
|-----------|---------|--------|-------------|
| `inscribed_radius` | `0.25` | Physical robot footprint radius (meters). Cells within this of lethal → also lethal. | Set to `robot_radius + safety_margin`. |
| `inflation_radius` | `0.8` | Soft cost buffer around obstacles (meters). | Increase (1.0-1.5m) to push paths further from walls/cliffs. |
| `inflation_cost_scaling` | `3.0` | Exponential decay rate of inflation cost buffer. | Increase (4.0-5.0) for sharper dropoff; decrease (1.0-2.0) for wider soft buffer. |
| `MAX_SAFE_SLOPE` | `0.5` | Steepness threshold for lethal hard-block. | Adjust based on vehicle gradeability. |
| `max_traversable_height_diff` | `0.30` | Max step height threshold in meters. | Adjust based on wheel/ground clearance. |

### D* Planner Parameters (`dstar_node` / `planner_server`)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `clearance_field_weight` | `3.0` | Geometric obstacle repulsion in D* `edgeCost` |
| `clearance_decay_distance` | `0.5` | Clearance field exponential decay distance |
| `terrain_weight` | `2.0` | Terrain difficulty weight |
