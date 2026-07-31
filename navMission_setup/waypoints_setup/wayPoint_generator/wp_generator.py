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
from scipy.ndimage import distance_transform_edt, label, binary_erosion

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
    # Optional ratios (defaults to None for backward compatibility)
    boundary_margin_ratio: Optional[float] = 0.05      # 5% of map size
    min_spacing_ratio: Optional[float] = 0.10            # 10% of map diagonal
    max_spacing_ratio: Optional[float] = 0.40            # 40% of map diagonal
    max_turn_angle_deg: Optional[float] = 80.0      # NEW
    min_start_end_dist: Optional[float] = 0.0       # NEW

# ==========================================
# HELPER: MASK GENERATION
# ==========================================

#helps waypoints stay inside irregular map shape

def _compute_valid_map_mask(costmaps: list, boundary_margin_cells: int = 5) -> np.ndarray:
    """Identifies black padding, removes it, and erodes the valid area 
        to enforce a strict safety margin away from map edges."""
    """Identifies -1 padding and erodes the valid area."""
    if not costmaps:
        return None
    h, w = costmaps[0].shape
    universal_mask = np.ones((h, w), dtype=bool)

    for cmap in costmaps: 
        # 1. Zero regions touching outer image borders are the black background padding
        # The costmaps use -1 for padding/unknown space
        padding_mask = (cmap == -1) | np.isnan(cmap)
        labeled_array, _ = label(padding_mask)
        
        border_labels = set()
        border_labels.update(labeled_array[0, :])
        border_labels.update(labeled_array[-1, :])
        border_labels.update(labeled_array[:, 0])
        border_labels.update(labeled_array[:, -1])
        border_labels.discard(0)
        
        map_mask = np.ones((h, w), dtype=bool)
        for lbl in border_labels:
            map_mask[labeled_array == lbl] = False  
            
        universal_mask &= map_mask
    # 2. Erode the mask inward by the boundary margin so waypoints stay away from edges
    if boundary_margin_cells > 0:
        struct = np.ones((3, 3), dtype=bool)
        universal_mask = binary_erosion(universal_mask, structure=struct, iterations=boundary_margin_cells)

    return universal_mask


def enforce_valid_waypoints(waypoints: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """
    Ensures every waypoint coordinate strictly lies within the valid map mask.
    If a waypoint falls outside, it snaps it to the nearest valid pixel inside the mask.
    """
    if valid_mask is None or waypoints.size == 0:
        return waypoints

    h, w = valid_mask.shape
    cleaned_waypoints = waypoints.copy()

    # Create an invalid distance map to penalize out-of-bounds searching
    invalid_distances = np.where(valid_mask, 0.0, np.inf)
    yy, xx = np.indices((h, w))

    # Flatten the first two dimensions to iterate through all individual points easily
    original_shape = cleaned_waypoints.shape
    pts = cleaned_waypoints.reshape(-1, 2)

    for i in range(len(pts)):
        x = int(round(pts[i, 0]))
        y = int(round(pts[i, 1]))

        # Clip to grid bounds first
        x_safe = np.clip(x, 0, w - 1)
        y_safe = np.clip(y, 0, h - 1)

        # If it falls outside the valid mask (e.g. in the black padding)
        if not valid_mask[y_safe, x_safe]:
            # Find the closest valid pixel within the mask using the distance map
            dist_field = np.sqrt((yy - y_safe)**2 + (xx - x_safe)**2) + invalid_distances
            min_y, min_x = np.unravel_index(np.argmin(dist_field), dist_field.shape)
            
            pts[i, 0] = min_x
            pts[i, 1] = min_y

    return pts.reshape(original_shape)

# ==========================================
# STEP 1: INPUT HANDLING (UNIVERSAL MULTI-MAP)
# ==========================================
class InputLoader:
    """Reads all available map inputs and merges obstacles for universal safety."""

    @staticmethod
    def load(inputs_dir: str = "inputs") -> Tuple[list, np.ndarray, RoverConfig, Constraints, list]:
        # Automatically resolve path relative to this script's directory
        script_dir = Path(__file__).resolve().parent
        base_path = script_dir / inputs_dir

        # 1. Load JSON Configs
        rover_file = base_path / "rover_config.json"
        assert rover_file.exists(), f"CRITICAL: Missing {rover_file}"
        with open(rover_file, "r") as f:
            rover_cfg = RoverConfig(**json.load(f))

        constraints_file = base_path / "wp_constraints.json"
        assert constraints_file.exists(), f"CRITICAL: Missing {constraints_file}"
        with open(constraints_file, "r") as f:
            constraints = Constraints(**json.load(f))

        # 2. Discover Maps & Initialize Master Obstacles
        costmaps = []
        map_names = []
        master_obstacles = None
        
        costmap_files = sorted(list(base_path.glob("costmap_*.npz")))
        assert len(costmap_files) > 0, "CRITICAL: No costmap files found in inputs directory!"

        for cmap_path in costmap_files:
            idx_str = cmap_path.stem.split('_')[-1]
            map_name = f"map_{idx_str}"
            map_names.append(map_name)

            # Load Costmap
            with np.load(cmap_path) as data:
                cmap = data["total"].astype(np.float32)
                costmaps.append(cmap)

                # NEW: pull the resolution the costmap was actually generated at
                stored_resolution = float(data["resolution"]) if "resolution" in data.files else None

            h, w = cmap.shape

            # Initialize spatial variables on the first pass
            if master_obstacles is None:
                # NEW: reconcile rover_cfg's resolution with what the costmap was actually built at
                if stored_resolution is not None and not math.isclose(
                    stored_resolution, rover_cfg.grid_resolution_m, rel_tol=1e-3
                ):
                    logging.warning(
                        f"grid_resolution_m mismatch: rover_config.json says "
                        f"{rover_cfg.grid_resolution_m} m/cell, but {cmap_path.name} was generated at "
                        f"{stored_resolution} m/cell. Using the costmap's stored resolution."
                    )
                    rover_cfg.grid_resolution_m = stored_resolution

                map_diag = math.sqrt(h**2 + w**2)
                boundary_ratio = getattr(constraints, "boundary_margin_ratio", 0.05)
                min_ratio = getattr(constraints, "min_spacing_ratio", 0.10)
                max_ratio = getattr(constraints, "max_spacing_ratio", 0.40)

                constraints.boundary_margin_cells = max(1, int(min(h, w) * boundary_ratio))
                constraints.min_spacing_cells = max(1.0, map_diag * min_ratio)
                constraints.max_spacing_cells = max(2.0, map_diag * max_ratio)

                master_obstacles = np.zeros((h, w), dtype=np.uint8)
                res = rover_cfg.grid_resolution_m
                center_x, center_y = w / 2.0, h / 2.0
                rock_radius_cells = max(1, int(round(0.5 / res)))
                grid_y, grid_x = np.ogrid[:h, :w]

            # 3. Load & Merge Obstacles
            obs_path = base_path / f"obstacle_data_{idx_str}.npy"
            assert obs_path.exists(), f"CRITICAL: Missing {obs_path.name} for {cmap_path.name}"
            
            obs_raw = np.load(obs_path, allow_pickle=True)
            if isinstance(obs_raw, np.ndarray) and obs_raw.dtype == object and obs_raw.ndim == 0:
                obs_raw = obs_raw.item()

            if isinstance(obs_raw, np.ndarray) and obs_raw.ndim == 2:
                current_obs = obs_raw.astype(np.uint8)
                master_obstacles = np.maximum(master_obstacles, current_obs) # Merge via logical OR
            else:
                items = obs_raw.flat if isinstance(obs_raw, np.ndarray) else obs_raw
                for obj in items:
                    # Updated to include BOTH collidable and barrier flags
                    if isinstance(obj, dict) and (obj.get("is_collidable", True) or obj.get("is_barrier", False)):
                        px = int(round(center_x + obj.get("x", 0.0) / res))
                        py = int(round(center_y + obj.get("y", 0.0) / res))
                        if 0 <= px < w and 0 <= py < h:
                            mask = (grid_x - px)**2 + (grid_y - py)**2 <= rock_radius_cells**2
                            master_obstacles[mask] = 1

        return costmaps, master_obstacles, rover_cfg, constraints, map_names

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
    """Fast candidate filtering using precomputed Euclidean Distance Transform 
    and a valid map mask to exclude out-of-bounds background padding."""

    def __init__(self, obstacles: np.ndarray, valid_mask: np.ndarray = None):
        # Precompute clearance map ONCE: O(H * W)
        # Free space = 1, Obstacles = 0
        #Ensure the filter treats both actual obstacles (> 0 or rocks) and the -1 padding mask as blocked space:
        free_space = (obstacles == 0).astype(np.uint8)
        if valid_mask is not None:
            free_space[~valid_mask] = 0
        self.clearance_map = distance_transform_edt(free_space)
        self.valid_mask = valid_mask

    def filter(self, candidates: np.ndarray, required_clearance_cells: int) -> np.ndarray:
        if candidates.size == 0:
            return candidates

        x = candidates[:, 0].astype(int)
        y = candidates[:, 1].astype(int)
        h, w = self.clearance_map.shape

        # 1. Ensure coordinates are within grid bounds
        in_bounds = (x >= 0) & (x < w) & (y >= 0) & (y < h)
        
        # 2. Check valid map boundary mask if provided
        if self.valid_mask is not None:
            mask_valid = np.zeros_like(in_bounds, dtype=bool)
            valid_indices_in_bounds = in_bounds
            mask_valid[valid_indices_in_bounds] = self.valid_mask[y[valid_indices_in_bounds], x[valid_indices_in_bounds]]
        else:
            mask_valid = in_bounds

        # 3. Check clearance requirements
        safe_x = np.clip(x, 0, w - 1)
        safe_y = np.clip(y, 0, h - 1)

        max_possible_clearance = float(np.max(self.clearance_map)) if self.clearance_map.size > 0 else 0.0
        effective_clearance = min(float(required_clearance_cells), max_possible_clearance)

        clearance_valid = (self.clearance_map[safe_y, safe_x] >= effective_clearance)

        # Combine all validity checks
        final_valid = mask_valid & clearance_valid

        if not np.any(final_valid):
            logging.warning(f"Clearance requirement ({required_clearance_cells}) exceeds available map clearance ({max_possible_clearance:.1f}) or all candidates fell outside valid map bounds.")

        return candidates[final_valid]
    
# ==========================================
# STEP 4: INCREMENTAL WAYPOINT BUILDER
# ==========================================
class waypointBuilder:
    """Builds 5-point waypoints incrementally using valid spatial step ranges."""

    def __init__(self, min_spacing: float, max_spacing: float, seed: Optional[int] = None,
                 max_turn_angle_deg: float = 80.0, min_start_end_dist: float = 0.0):  # ← add these two params
        self.min_spacing = min_spacing
        self.max_spacing = max_spacing
        self.rng = np.random.default_rng(seed)
        self.max_turn_angle_deg = max_turn_angle_deg  #NEW
        self.min_start_end_dist = min_start_end_dist    #NEW
        self._cos_limit = math.cos(math.radians(max_turn_angle_deg)) #NEW

    def build_waypoints(self, candidates: np.ndarray, target_count: int, max_attempts: int) -> np.ndarray:
        if len(candidates) < 5:
            logging.warning("Fewer than 5 candidates available. Cannot build 5-point waypoints.")
            return np.array([])

        waypoints = []
        num_candidates = len(candidates)
        attempts = 0
        fail_dead_end = 0        # NEW: counts walks that died from spacing/angle constraints
        fail_displacement = 0    # NEW: counts walks that finished but were too short start-to-end

        while len(waypoints) < target_count and attempts < max_attempts:
            attempts += 1
            start_idx = self.rng.integers(0, num_candidates)
            wp = [candidates[start_idx]]
            visited_indices = {start_idx}
            prev_vec = None
            
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

                # Enforce turn angle constraint if applicable
                if prev_vec is not None and unvisited_indices:
                    candidate_vecs = candidates[unvisited_indices] - curr_pt
                    norms = np.linalg.norm(candidate_vecs, axis=1)
                    cos_angles = (candidate_vecs @ prev_vec) / (norms * np.linalg.norm(prev_vec))
                    unvisited_indices = [idx for idx, ca in zip(unvisited_indices, cos_angles) if ca >= self._cos_limit]

                # If no valid candidates remain, break early
                if not unvisited_indices:
                    success = False
                    fail_dead_end += 1  # NEW
                    break  # Dead end, discard walk

                next_idx = self.rng.choice(unvisited_indices)
                visited_indices.add(next_idx)
                next_pt = candidates[next_idx]
                prev_vec = next_pt - curr_pt
                wp.append(next_pt)

            if success:
                # Check net displacement constraint if applicable
                net_disp = np.linalg.norm(wp[-1] - wp[0])
                if net_disp >= self.min_start_end_dist:
                    waypoints.append(wp)
                else:
                    fail_displacement += 1  # NEW

        # NEW: summary so you can tell which constraint is starving the generator
        logging.info(f"build_waypoints: {attempts} attempts | {len(waypoints)} accepted | "
                     f"{fail_dead_end} dead-ended (spacing/angle) | {fail_displacement} rejected on start-end distance")

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
# STEP 7: DIFFICULTY RANKER (AVERAGE COST)
# ==========================================
class DifficultyRanker:
    """Ranks waypoints by evaluating their sum of costs across all provided maps, 
    sorting by the average cost."""

    @staticmethod
    def rank(waypoints: np.ndarray, costmaps: list) -> Tuple[np.ndarray, np.ndarray, list]:
        if len(waypoints) == 0:
            return waypoints, np.array([]), []

        avg_scores = []
        detailed_scores = []

        for wp in waypoints:
            x_coords, y_coords = wp[:, 0], wp[:, 1]
            
            wp_map_scores = []
            for cmap in costmaps:
                # Sum of terrain costs for this specific map
                score = float(np.sum(cmap[y_coords, x_coords]))
                wp_map_scores.append(score)
                
            avg_score = float(np.mean(wp_map_scores))
            avg_scores.append(avg_score)
            detailed_scores.append(wp_map_scores)

        avg_scores_arr = np.array(avg_scores, dtype=np.float32)
        
        # Sort ascending (Easiest -> Hardest) based on the Average
        sort_indices = np.argsort(avg_scores_arr)
        
        sorted_waypoints = waypoints[sort_indices]
        sorted_avg_scores = avg_scores_arr[sort_indices]
        sorted_detailed = [detailed_scores[i] for i in sort_indices]

        return sorted_waypoints, sorted_avg_scores, sorted_detailed


# ==========================================
# STEP 8: EXPORTER WITH METADATA
# ==========================================
class Exporter:
    """Persists array outputs and logs detailed multi-map scores."""

    @staticmethod
    def _make_serializable(val):
        """Recursively converts numpy data types to native Python types for JSON compatibility."""
        if isinstance(val, dict):
            return {str(k): Exporter._make_serializable(v) for k, v in val.items()}
        elif isinstance(val, list):
            return [Exporter._make_serializable(v) for v in val]
        elif isinstance(val, tuple):
            return [Exporter._make_serializable(v) for v in val]
        elif isinstance(val, (np.float32, np.float64, np.floating)):
            return float(val)
        elif isinstance(val, (np.int32, np.int64, np.integer)):
            return int(val)
        elif isinstance(val, np.ndarray):
            return val.tolist()
        else:
            return val

    @staticmethod
    def export(
        waypoints: np.ndarray,
        avg_scores: np.ndarray,
        detailed_scores: list,
        map_names: list,
        rover_cfg: RoverConfig,
        constraints: Constraints,
        outputs_dir: str = "outputs"
    ):
        out_path = Path(outputs_dir)
        out_path.mkdir(parents=True, exist_ok=True)

        for old_file in out_path.glob("wp*_*.npy"):
            old_file.unlink()

        export_details = []
        for i, (wp_set, avg, d_scores) in enumerate(zip(waypoints, avg_scores, detailed_scores)):
            avg_val = float(avg)
            filename = f"wp{i:02d}_{int(round(avg_val))}.npy"
            np.save(out_path / filename, wp_set)
            
            # Map the separate scores to their respective map names for the JSON
            score_dict = {"average": avg_val}
            for m_name, s in zip(map_names, d_scores):
                score_dict[m_name] = float(s)
                
            export_details.append({
                "id": f"wp{i:02d}",
                "file": filename,
                "scores": score_dict
            })

        metadata = {
            "generator_version": "2.1.0",
            "timestamp": datetime.now().isoformat(),
            "maps_processed": map_names,
            "wp_count": int(len(waypoints)),
            "difficulty_stats_average": {
                "min": float(np.min(avg_scores)) if len(avg_scores) > 0 else 0.0,
                "max": float(np.max(avg_scores)) if len(avg_scores) > 0 else 0.0,
                "mean": float(np.mean(avg_scores)) if len(avg_scores) > 0 else 0.0,
            },
            "waypoint_breakdown": export_details,
            "rover_config": rover_cfg.__dict__,
            "constraints": constraints.__dict__
        }

        # Ensure complete JSON serialization safety
        clean_metadata = Exporter._make_serializable(metadata)

        with open(out_path / "generation_log.json", "w") as f:
            json.dump(clean_metadata, f, indent=4)

        logging.info(f"Successfully exported {len(waypoints)} universal waypoint files to '{out_path.resolve()}'")
        
# ==========================================
# INTERFACE / PIPELINE ORCHESTRATOR
# ==========================================
def run_generator():
    start_time = time.time()
    logging.info("--- Starting Universal Waypoint Generator Pipeline ---")

    # Step 1: Load Maps
    costmaps, master_obstacles, rover_cfg, constraints, map_names = InputLoader.load("inputs")
    logging.info(f"Loaded {len(costmaps)} maps | Master Obstacle Grid: {master_obstacles.shape}")

    # NEW: Generate the valid mask; avoid waypoints out of irrigular map
    valid_map_mask = _compute_valid_map_mask(costmaps, constraints.boundary_margin_cells)

    # Step 2: Sample Candidates
    sampler = CandidateSampler(master_obstacles.shape, constraints.boundary_margin_cells, constraints.seed)
    raw_candidates = sampler.sample_poisson(max(1.0, constraints.min_spacing_cells * 0.5), constraints.candidate_count)
    
# Step 3: Filter Candidates (Now enforcing the valid_map_mask!)
    candidate_filter = CandidateFilter(master_obstacles, valid_mask=valid_map_mask)
    max_clearance_available = float(np.max(candidate_filter.clearance_map)) if candidate_filter.clearance_map.size > 0 else 0.0
    logging.info(f"Required clearance: {rover_cfg.clearance_radius_cells} cells | "
                 f"Max clearance available on map: {max_clearance_available:.1f} cells")  # NEW
    valid_candidates = candidate_filter.filter(raw_candidates, rover_cfg.clearance_radius_cells)
    logging.info(f"Valid candidates: {len(valid_candidates)} | min_spacing={constraints.min_spacing_cells:.1f} | "
                 f"max_spacing={constraints.max_spacing_cells:.1f} | max_turn_angle={constraints.max_turn_angle_deg} | "
                 f"min_start_end_dist={constraints.min_start_end_dist}")  # NEW

    if len(valid_candidates) < 5:
        logging.error("Map bounds too constrained across all maps. Aborting gracefully.")
        Exporter.export(np.array([]), np.array([]), [], map_names, rover_cfg, constraints, "outputs")
        return

    # Step 4: Build Waypoints
    builder = waypointBuilder(
        constraints.min_spacing_cells,
        constraints.max_spacing_cells,
        constraints.seed,
        max_turn_angle_deg=constraints.max_turn_angle_deg,
        min_start_end_dist=constraints.min_start_end_dist,
    )
    raw_waypoints = builder.build_waypoints(valid_candidates, constraints.target_wp_count * 2, constraints.max_attempts)

    # Step 5: Validate
    validated_waypoints = waypointValidator.validate(raw_waypoints, constraints.min_spacing_cells)

    # Step 6: Deduplicate
    unique_waypoints = SimilarityFilter.deduplicate(validated_waypoints, constraints.duplicate_distance_threshold)

    # NEW: Actively snap stray points securely into valid terrain
    snapped_waypoints = enforce_valid_waypoints(unique_waypoints, valid_map_mask)


    # Step 7: Multi-Map Ranking
    ranked_waypoints, avg_scores, detailed_scores = DifficultyRanker.rank(snapped_waypoints, costmaps)
    
    # Stratified Sampling across the full difficulty spectrum
    total_found = len(ranked_waypoints)
    target_count = constraints.target_wp_count

    if total_found > target_count:
        # Pick indices evenly spaced from index 0 (easiest) to index total_found-1 (hardest)
        selected_indices = np.linspace(0, total_found - 1, target_count, dtype=int)
        
        selected_waypoints = ranked_waypoints[selected_indices]
        selected_avg_scores = avg_scores[selected_indices]
        selected_detailed_scores = [detailed_scores[i] for i in selected_indices]
    else:
        selected_waypoints = ranked_waypoints
        selected_avg_scores = avg_scores
        selected_detailed_scores = detailed_scores

    # Step 8: Export
    Exporter.export(
        selected_waypoints, 
        selected_avg_scores, 
        selected_detailed_scores, 
        map_names, rover_cfg, constraints, "outputs"
    )

    elapsed = time.time() - start_time
    logging.info(f"--- Pipeline Finished in {elapsed:.3f}s ---")

if __name__ == "__main__":
    run_generator()