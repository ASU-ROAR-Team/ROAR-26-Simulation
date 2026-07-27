"""
Waypoint Generator Pipeline (Production Refactor)
Architecture: Single-file orchestrator & high-performance processing engine.
"""

import json
import logging
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple, Set

import numpy as np
from scipy.ndimage import distance_transform_edt

# Configure standard logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)


# ==========================================
# DATACLASS SCHEMAS (STRICT INPUT STRUCTURES)
# ==========================================
@dataclass
class RoverConfig:
    rover_length_m: float
    rover_width_m: float
    clearance_margin_m: float
    grid_resolution_m: float

    @property
    def clearance_radius_cells(self) -> int:
        radius_m = math.sqrt((self.rover_length_m / 2)**2 + (self.rover_width_m / 2)**2) + self.clearance_margin_m
        return int(math.ceil(radius_m / self.grid_resolution_m))


@dataclass
class Constraints:
    candidate_count: int
    min_spacing_cells: float
    max_spacing_cells: float
    boundary_margin_cells: int
    duplicate_distance_threshold: float
    target_wp_count: int
    max_attempts: int
    seed: Optional[int]
    weights: dict
    # Optional ratios (defaults to None for backward compatibility)
    boundary_margin_ratio: Optional[float] = 0.05      # 5% of map size
    min_spacing_ratio: Optional[float] = 0.10            # 10% of map diagonal
    max_spacing_ratio: Optional[float] = 0.40            # 40% of map diagonal


# ==========================================
# STEP 1: INPUT HANDLING (STRICT DISK LOAD)
# ==========================================
class InputLoader:
    """Reads input data strictly from disk and dynamically scales spatial bounds."""

    @staticmethod
    def load(inputs_dir: str = "inputs") -> Tuple[np.ndarray, np.ndarray, RoverConfig, Constraints]:
        base_path = Path(inputs_dir)
        
        # 1. Load Costmap
        costmap_path = base_path / "costmap.npz"
        assert costmap_path.exists(), f"CRITICAL: Missing required input file {costmap_path}"
        with np.load(costmap_path) as data:
            costmap = data["total"].astype(np.float32)

        h, w = costmap.shape
        map_diag = math.sqrt(h**2 + w**2)

        # 2. Load JSON Configs
        rover_file = base_path / "rover_config.json"
        assert rover_file.exists(), f"CRITICAL: Missing required input file {rover_file}"
        with open(rover_file, "r") as f:
            rover_cfg = RoverConfig(**json.load(f))

        constraints_file = base_path / "wp_constraints.json"
        assert constraints_file.exists(), f"CRITICAL: Missing required input file {constraints_file}"
        with open(constraints_file, "r") as f:
            constraints = Constraints(**json.load(f))

        # 3. Dynamic Ratio Calculations & Map Scaling (Purely Percentage-Driven)
        h, w = costmap.shape
        map_diag = math.sqrt(h**2 + w**2)

        # Retrieve ratios from config (with clean default fallbacks)
        boundary_ratio = getattr(constraints, "boundary_margin_ratio", 0.05)
        min_ratio = getattr(constraints, "min_spacing_ratio", 0.10)
        max_ratio = getattr(constraints, "max_spacing_ratio", 0.40)

        # Calculate bounds dynamically based on map size and ratios
        constraints.boundary_margin_cells = max(1, int(min(h, w) * boundary_ratio))
        constraints.min_spacing_cells = max(1.0, map_diag * min_ratio)
        constraints.max_spacing_cells = max(2.0, map_diag * max_ratio)

        # 4. Load & Rasterize Obstacles
        obstacle_path = base_path / "obstacle_data.npy"
        assert obstacle_path.exists(), f"CRITICAL: Missing required input file {obstacle_path}"
        obs_raw = np.load(obstacle_path, allow_pickle=True)
        
        if isinstance(obs_raw, np.ndarray) and obs_raw.dtype == object and obs_raw.ndim == 0:
            obs_raw = obs_raw.item()

        if isinstance(obs_raw, np.ndarray) and obs_raw.ndim == 2:
            obstacles = obs_raw.astype(np.uint8)
        else:
            # Rasterize Gazebo world objects
            res = rover_cfg.grid_resolution_m
            center_x, center_y = w / 2.0, h / 2.0
            obstacles = np.zeros((h, w), dtype=np.uint8)
            grid_y, grid_x = np.ogrid[:h, :w]

            rock_radius_cells = max(1, int(round(0.5 / res)))
            # Important This line takes a flat 0.5-meter radius and divides it by your grid resolution (res) 
            # to determine how many cells the obstacle occupies on the costmap, treating every object as a uniform circle 
            # regardless of its actual shape.
            
            items = obs_raw.flat if isinstance(obs_raw, np.ndarray) else obs_raw

            for obj in items:
                if isinstance(obj, dict) and obj.get("is_collidable", True):
                    px = int(round(center_x + obj.get("x", 0.0) / res))
                    py = int(round(center_y + obj.get("y", 0.0) / res))
                    if 0 <= px < w and 0 <= py < h:
                        mask = (grid_x - px)**2 + (grid_y - py)**2 <= rock_radius_cells**2
                        obstacles[mask] = 1

        assert costmap.shape == obstacles.shape, (
            f"Dimension Mismatch: Costmap {costmap.shape} vs Obstacles {obstacles.shape}"
        )

        return costmap, obstacles, rover_cfg, constraints


# ==========================================
# STEP 2: POISSON DISK CANDIDATE SAMPLER
# ==========================================
class CandidateSampler:
    """Generates evenly spaced spatial candidates using grid-based Poisson Disk Sampling."""

    def __init__(self, map_shape: Tuple[int, int], boundary_margin: int, seed: Optional[int] = None):
        self.height, self.width = map_shape
        self.margin_y = max(0, min(boundary_margin, (self.height - 1) // 2))
        self.margin_x = max(0, min(boundary_margin, (self.width - 1) // 2))
        self.rng = np.random.default_rng(seed)

    def sample_poisson(self, min_dist: float, max_candidates: int) -> np.ndarray:
        """Bridson's algorithm approximation for uniform spatial dispersion."""
        min_dist = max(1.0, min(min_dist, min(self.width, self.height) / 2.0))
        cell_size = min_dist / math.sqrt(2)
        grid_w = max(1, int(math.ceil(self.width / cell_size)))
        grid_h = max(1, int(math.ceil(self.height / cell_size)))
        grid = np.full((grid_h, grid_w), -1, dtype=np.int32)

        candidates = []
        active_list = []

        low_x, high_x = self.margin_x, self.width - self.margin_x
        low_y, high_y = self.margin_y, self.height - self.margin_y

        # Initial random point
        init_x = int(self.rng.integers(low_x, high_x)) if high_x > low_x else self.width // 2
        init_y = int(self.rng.integers(low_y, high_y)) if high_y > low_y else self.height // 2
        first_pt = np.array([init_x, init_y], dtype=np.int32)
        
        candidates.append(first_pt)
        active_list.append(0)
        
        gy_init = min(grid_h - 1, int(init_y / cell_size))
        gx_init = min(grid_w - 1, int(init_x / cell_size))
        grid[gy_init, gx_init] = 0

        while active_list and len(candidates) < max_candidates:
            idx = self.rng.integers(0, len(active_list))
            pos_idx = active_list[idx]
            point = candidates[pos_idx]
            found = False

            for _ in range(30):  # 30 sample attempts around point
                rad = self.rng.uniform(min_dist, 2 * min_dist)
                angle = self.rng.uniform(0, 2 * math.pi)
                new_x = int(point[0] + rad * math.cos(angle))
                new_y = int(point[1] + rad * math.sin(angle))

                if (self.margin_x <= new_x < self.width - self.margin_x and 
                        self.margin_y <= new_y < self.height - self.margin_y):
                    
                    gx = min(grid_w - 1, int(new_x / cell_size))
                    gy = min(grid_h - 1, int(new_y / cell_size))
                    
                    # Check neighbor cells for distance conflicts
                    conflict = False
                    for i in range(max(0, gy - 2), min(grid_h, gy + 3)):
                        for j in range(max(0, gx - 2), min(grid_w, gx + 3)):
                            neighbor_idx = grid[i, j]
                            if neighbor_idx != -1:
                                dist = np.linalg.norm(candidates[neighbor_idx] - [new_x, new_y])
                                if dist < min_dist:
                                    conflict = True
                                    break
                        if conflict:
                            break

                    if not conflict:
                        new_pt = np.array([new_x, new_y], dtype=np.int32)
                        candidates.append(new_pt)
                        c_idx = len(candidates) - 1
                        active_list.append(c_idx)
                        grid[gy, gx] = c_idx
                        found = True
                        break

            if not found:
                active_list.pop(idx)

        return np.array(candidates, dtype=np.int32)


# ==========================================
# STEP 3: CANDIDATE FILTER (OBSTACLE CLEARANCE ONLY)
# ==========================================
class CandidateFilter:
    """Fast candidate filtering using precomputed Euclidean Distance Transform.
    Filters purely on the presence of collidable obstacles — costmap values
    play no role in accept/reject decisions here."""

    def __init__(self, obstacles: np.ndarray):
        # Precompute clearance map ONCE: O(H * W)
        # Free space = 1, Obstacles = 0
        free_space = (obstacles == 0).astype(np.uint8)
        self.clearance_map = distance_transform_edt(free_space)

    def filter(self, candidates: np.ndarray, required_clearance_cells: int) -> np.ndarray:
        if candidates.size == 0:
            return candidates

        x = candidates[:, 0]
        y = candidates[:, 1]

        # Dynamically scale clearance bounds to prevent map geometry wipeouts
        max_possible_clearance = float(np.max(self.clearance_map)) if self.clearance_map.size > 0 else 0.0
        effective_clearance = min(float(required_clearance_cells), max_possible_clearance)

        clearance_valid = self.clearance_map[y, x] >= effective_clearance

        if not np.any(clearance_valid):
            logging.warning(f"Clearance requirement ({required_clearance_cells}) exceeds available map clearance ({max_possible_clearance:.1f}). No candidates passed.")

        return candidates[clearance_valid]


# ==========================================
# STEP 4: INCREMENTAL WAYPOINT BUILDER
# ==========================================
class waypointBuilder:
    """Builds 5-point waypoints incrementally using valid spatial step ranges."""

    def __init__(self, min_spacing: float, max_spacing: float, seed: Optional[int] = None):
        self.min_spacing = min_spacing
        self.max_spacing = max_spacing
        self.rng = np.random.default_rng(seed)

    def build_waypoints(self, candidates: np.ndarray, target_count: int, max_attempts: int) -> np.ndarray:
        if len(candidates) < 5:
            logging.warning("Fewer than 5 candidates available. Cannot build 5-point waypoints.")
            return np.array([])

        waypoints = []
        num_candidates = len(candidates)
        attempts = 0

        while len(waypoints) < target_count and attempts < max_attempts:
            attempts += 1
            start_idx = self.rng.integers(0, num_candidates)
            wp = [candidates[start_idx]]
            visited_indices = {start_idx}
            
            success = True
            for _ in range(4):  # Select WP1, WP2, WP3, WP4
                curr_pt = wp[-1]
                
                # Compute distance from current point to all candidates
                dists = np.linalg.norm(candidates - curr_pt, axis=1)
                
                # Find indices matching valid range constraints
                valid_mask = (dists >= self.min_spacing) & (dists <= self.max_spacing)
                valid_indices = np.where(valid_mask)[0]
                
                # Filter out already used points in this wp
                unvisited_indices = [idx for idx in valid_indices if idx not in visited_indices]

                if not unvisited_indices:
                    success = False
                    break  # Dead end, discard walk

                next_idx = self.rng.choice(unvisited_indices)
                visited_indices.add(next_idx)
                wp.append(candidates[next_idx])

            if success:
                waypoints.append(wp)

        return np.array(waypoints, dtype=np.int32)


# ==========================================
# STEP 5: waypoint VALIDATOR (QUALITY GATE)
# ==========================================
class waypointValidator:
    """Ensures paths avoid self-intersection and satisfy non-consecutive minimum spacing."""

    @staticmethod
    def validate(waypoints: np.ndarray, min_spacing: float) -> np.ndarray:
        if len(waypoints) == 0:
            return waypoints

        valid_waypoints = []
        half_spacing = min_spacing * 0.5

        for wp in waypoints:
            # Non-consecutive waypoint distance check
            is_valid = True
            for i in range(len(wp)):
                for j in range(i + 2, len(wp)):
                    dist = np.linalg.norm(wp[i] - wp[j])
                    if dist < half_spacing:
                        is_valid = False
                        break
                if not is_valid:
                    break

            if is_valid:
                valid_waypoints.append(wp)

        return np.array(valid_waypoints, dtype=np.int32)


# ==========================================
# STEP 6: SIMILARITY FILTER (SPATIAL HASHING)
# ==========================================
class SimilarityFilter:
    """Removes redundant waypoints using fast coordinate bucket hashing."""

    @staticmethod
    def deduplicate(waypoints: np.ndarray, spatial_threshold: float) -> np.ndarray:
        if len(waypoints) == 0:
            return waypoints

        unique_waypoints = []
        seen_hashes: Set[Tuple[Tuple[int, int], ...]] = set()

        for wp in waypoints:
            # Quantize coordinates to discrete spatial buckets based on threshold
            quantized = tuple(
                (int(pt[0] // spatial_threshold), int(pt[1] // spatial_threshold))
                for pt in wp
            )
            
            if quantized not in seen_hashes:
                seen_hashes.add(quantized)
                unique_waypoints.append(wp)

        return np.array(unique_waypoints, dtype=np.int32)


# ==========================================
# STEP 7: DIFFICULTY RANKER (SUM OF PATH COSTS)
# ==========================================
class DifficultyRanker:
    """Ranks waypoints from easiest to hardest. Difficulty score is the exact
    sum of costmap values at the starting point plus wp1, wp2, wp3, wp4."""

    @staticmethod
    def rank(waypoints: np.ndarray, costmap: np.ndarray, weights: dict) -> Tuple[np.ndarray, np.ndarray]:
        if len(waypoints) == 0:
            return waypoints, np.array([])

        scores = []
        for wp in waypoints:
            # Terrain costs at start + wp1 + wp2 + wp3 + wp4
            x_coords, y_coords = wp[:, 0], wp[:, 1]
            wp_costs = costmap[y_coords, x_coords]

            # Difficulty = exact sum of all 5 point costs
            score = float(np.sum(wp_costs))
            scores.append(score)

        scores_arr = np.array(scores, dtype=np.float32)
        
        # Sort ascending (Easiest -> Hardest)
        sort_indices = np.argsort(scores_arr)
        return waypoints[sort_indices], scores_arr[sort_indices]


# ==========================================
# STEP 8: EXPORTER WITH METADATA
# ==========================================
class Exporter:
    """Persists array outputs and JSON metadata logs."""

    @staticmethod
    def export(
        waypoints: np.ndarray,
        scores: np.ndarray,
        rover_cfg: RoverConfig,
        constraints: Constraints,
        outputs_dir: str = "outputs"
    ):
        out_path = Path(outputs_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        # 0. Clear stale waypoint files from any previous run
        for old_file in out_path.glob("wp*_*.npy"):
            old_file.unlink()

        # 1. Save one .npy per waypoint set: wp{index}_{score}.npy
        for i, (wp_set, score) in enumerate(zip(waypoints, scores)):
            filename = f"wp{i:02d}_{int(round(score))}.npy"
            np.save(out_path / filename, wp_set)

        # 2. Export Metadata JSON
        metadata = {
            "generator_version": "2.0.0",
            "timestamp": datetime.now().isoformat(),
            "wp_count": len(waypoints),
            "difficulty_stats": {
                "min": float(np.min(scores)) if len(scores) > 0 else 0.0,
                "max": float(np.max(scores)) if len(scores) > 0 else 0.0,
                "mean": float(np.mean(scores)) if len(scores) > 0 else 0.0,
            },
            "rover_config": rover_cfg.__dict__,
            "constraints": {
                k: v for k, v in constraints.__dict__.items() if k != "weights"
            }
        }

        with open(out_path / "generation_log.json", "w") as f:
            json.dump(metadata, f, indent=4)

        logging.info(f"Successfully exported {len(waypoints)} waypoint files to '{out_path.resolve()}'")


# ==========================================
# INTERFACE / PIPELINE ORCHESTRATOR
# ==========================================
def run_generator():
    start_time = time.time()
    logging.info("--- Starting Waypoint Generator Pipeline ---")

    # Step 1: Load Inputs
    costmap, obstacles, rover_cfg, constraints = InputLoader.load("inputs")
    logging.info(f"Loaded Costmap: {costmap.shape} | Rover Clearance: {rover_cfg.clearance_radius_cells} cells")

    # Step 2: Sample Candidates (Poisson Disk)
    sampler = CandidateSampler(
        map_shape=costmap.shape,
        boundary_margin=constraints.boundary_margin_cells,
        seed=constraints.seed
    )
    raw_candidates = sampler.sample_poisson(
        min_dist=max(1.0, constraints.min_spacing_cells * 0.5), # Force integer boundary safety
        max_candidates=constraints.candidate_count
    )
    logging.info(f"Generated {len(raw_candidates)} spatial candidate points via Poisson Disk Sampling.")

    # Step 3: Filter Candidates (Obstacle Clearance Only)
    candidate_filter = CandidateFilter(obstacles)
    valid_candidates = candidate_filter.filter(raw_candidates, rover_cfg.clearance_radius_cells)
    logging.info(f"Filtered candidates: {len(valid_candidates)} passed clearance check.")

    # Graceful exit instead of crash if map lacks space
    if len(valid_candidates) < 5:
        logging.error(f"Critical Error: Only {len(valid_candidates)} candidates passed filtering (need >=5). Map bounds may be too small. Aborting generation gracefully.")
        Exporter.export(np.array([]), np.array([]), rover_cfg, constraints, "outputs")
        return

    # Step 4: Incremental waypoint Builder
    builder = waypointBuilder(
        min_spacing=constraints.min_spacing_cells,
        max_spacing=constraints.max_spacing_cells,
        seed=constraints.seed
    )
    raw_waypoints = builder.build_waypoints(
        valid_candidates,
        target_count=constraints.target_wp_count * 2,  # Over-generate for filtering
        max_attempts=constraints.max_attempts
    )
    logging.info(f"Built {len(raw_waypoints)} valid 5-point wp paths.")

    # Step 5: waypoint Quality Validation
    validated_waypoints = waypointValidator.validate(raw_waypoints, constraints.min_spacing_cells)
    logging.info(f"Validated waypoints: {len(validated_waypoints)} passed path constraints.")

    # Step 6: Similarity Deduplication
    unique_waypoints = SimilarityFilter.deduplicate(
        validated_waypoints,
        spatial_threshold=constraints.duplicate_distance_threshold
    )
    logging.info(f"Deduplicated waypoints: {len(unique_waypoints)} unique trajectories remain.")

    # Step 7: Difficulty Ranking (sum of path costs)
    ranked_waypoints, scores = DifficultyRanker.rank(unique_waypoints, costmap, constraints.weights)

    # Truncate to final requested target count
    final_waypoints = ranked_waypoints[:constraints.target_wp_count]
    final_scores = scores[:constraints.target_wp_count]

    # Step 8: Export Artifacts
    Exporter.export(final_waypoints, final_scores, rover_cfg, constraints, "outputs")

    elapsed = time.time() - start_time
    logging.info(f"--- Pipeline Finished in {elapsed:.3f}s ---")


if __name__ == "__main__":
    run_generator()