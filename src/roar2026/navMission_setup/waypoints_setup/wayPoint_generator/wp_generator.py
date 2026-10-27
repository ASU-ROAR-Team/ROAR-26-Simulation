# """
# Waypoint Generator Pipeline (Refactored: No Difficulty Scoring)
# Architecture: Single-file orchestrator with high-performance processing.
# """

import json
import logging
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple, List, Set

import numpy as np
from scipy.ndimage import distance_transform_edt, label, binary_erosion

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)

# ====================================================
# DATACLASS SCHEMAS (STRICT INPUT STRUCTURES)
# ====================================================

@dataclass
class RoverConfig:
    rover_length_m: float
    rover_width_m: float
    clearance_margin_m: float
    grid_resolution_m: float

    @property
    def clearance_radius_cells(self) -> int:
        radius_m = math.sqrt((self.rover_length_m / 2) ** 2 + (self.rover_width_m / 2) ** 2) + self.clearance_margin_m
        return int(math.ceil(radius_m / self.grid_resolution_m))


@dataclass
class Constraints:
    candidate_count: int
    min_spacing_cells: float
    max_spacing_cells: float
    boundary_margin_cells: int
    duplicate_distance_threshold: float
    # New semantic fields
    points_per_mission: int = 5
    target_mission_count: int = 300
    # Backward‑compatible field (deprecated but kept for older pipelines)
    target_wp_count: int = 5
    max_attempts: int = 1000
    seed: Optional[int] = None
    # Optional ratios (preserve historic behavior)
    boundary_margin_ratio: Optional[float] = 0.05
    min_spacing_ratio: Optional[float] = 0.10
    max_spacing_ratio: Optional[float] = 0.40
    max_turn_angle_deg: Optional[float] = 80.0
    min_start_end_dist: Optional[float] = 0.0

# ====================================================
# HELPER: MASK GENERATION
# ====================================================

def _compute_valid_map_mask(heightmaps: list, boundary_margin_cells: int = 5) -> np.ndarray:
    """Create a mask of traversable cells, removing padding and applying a safety margin."""
    if not heightmaps:
        return None
    h, w = heightmaps[0].shape
    universal_mask = np.ones((h, w), dtype=bool)
    for cmap in heightmaps:
        padding_mask = (cmap == -1) | np.isnan(cmap) | (cmap <= -1.49)
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
    if boundary_margin_cells > 0:
        struct = np.ones((3, 3), dtype=bool)
        universal_mask = binary_erosion(universal_mask, structure=struct, iterations=boundary_margin_cells)
    return universal_mask


def enforce_valid_waypoints(waypoints: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """Snap any waypoint that lies outside the valid mask to the nearest valid cell."""
    if valid_mask is None or waypoints.size == 0:
        return waypoints
    h, w = valid_mask.shape
    cleaned = waypoints.copy()
    yy, xx = np.indices((h, w))
    for i in range(cleaned.shape[0]):
        for j in range(cleaned.shape[1]):
            x, y = int(round(cleaned[i, j, 0])), int(round(cleaned[i, j, 1]))
            x = np.clip(x, 0, w - 1)
            y = np.clip(y, 0, h - 1)
            if not valid_mask[y, x]:
                dist = np.sqrt((yy - y) ** 2 + (xx - x) ** 2)
                min_y, min_x = np.unravel_index(np.argmin(dist + (~valid_mask) * 1e6), valid_mask.shape)
                cleaned[i, j, 0] = min_x
                cleaned[i, j, 1] = min_y
    return cleaned

# ====================================================
# STEP 1: INPUT HANDLING (UNIVERSAL MULTI‑MAP)
# ====================================================

class InputLoader:
    """Loads rover configuration, constraints, and map data (heightmaps + obstacles)."""

    @staticmethod
    def load(inputs_dir: str = "inputs") -> Tuple[list, np.ndarray, RoverConfig, Constraints, List[str]]:
        script_dir = Path(__file__).resolve().parent
        base_path = script_dir / inputs_dir
        rover_file = base_path / "rover_config.json"
        assert rover_file.exists(), f"CRITICAL: Missing {rover_file}"
        with open(rover_file, "r") as f:
            rover_cfg = RoverConfig(**json.load(f))
        constraints_file = base_path / "wp_constraints.json"
        assert constraints_file.exists(), f"CRITICAL: Missing {constraints_file}"
        with open(constraints_file, "r") as f:
            constraints = Constraints(**json.load(f))
        # Load heightmap (used for map dimensions and mask)
        heightmap_path = base_path / "heightmap.npz"
        assert heightmap_path.exists(), f"CRITICAL: Missing {heightmap_path}"
        with np.load(heightmap_path) as data:
            heightmap = None
            for key in data.files:
                arr = data[key]
                if isinstance(arr, np.ndarray) and arr.ndim == 2:
                    heightmap = arr.astype(np.float32)
                    break
        assert heightmap is not None, "CRITICAL: Heightmap 2-D array not found in NPZ"

        # Load obstacle data (may be multiple files)
        master_obstacles = None
        obstacle_files = sorted(list(base_path.glob("obstacle_data*.npy")))
        assert obstacle_files, "CRITICAL: No obstacle files found in inputs directory!"
        for obs_path in obstacle_files:
            obs_raw = np.load(obs_path, allow_pickle=True)
            if isinstance(obs_raw, np.ndarray) and obs_raw.dtype == object and obs_raw.ndim == 0:
                obs_raw = obs_raw.item()
            if isinstance(obs_raw, np.ndarray) and obs_raw.ndim == 2:
                current_obs = obs_raw.astype(np.uint8)
                if master_obstacles is None:
                    master_obstacles = current_obs.copy()
                else:
                    master_obstacles = np.maximum(master_obstacles, current_obs)
            else:
                # Legacy obstacle format (list of dicts)
                items = obs_raw.flat if isinstance(obs_raw, np.ndarray) else obs_raw
                ox, oy = -18.431461, -7.687172
                res = getattr(rover_cfg, "grid_resolution_m", 0.25)
                h, w = heightmap.shape
                for obj in items:
                    if isinstance(obj, dict) and (obj.get("is_collidable", True) or obj.get("is_barrier", False)):
                        px = int(round((obj.get("x", 0.0) - ox) / res))
                        py = int(round((obj.get("y", 0.0) - oy) / res))
                        if 0 <= px < w and 0 <= py < h:
                            if obj.get("mesh_id") == -1:
                                # ArUco markers need a much larger safety buffer (e.g., 2.0m radius)
                                rock_radius_cells = max(1, int(round(2.0 / res)))
                            else:
                                # Standard rock buffer (0.5m radius)
                                rock_radius_cells = max(1, int(round(0.5 / res)))
                            grid_y, grid_x = np.ogrid[:h, :w]
                            mask = (grid_x - px) ** 2 + (grid_y - py) ** 2 <= rock_radius_cells ** 2
                            if master_obstacles is None:
                                master_obstacles = np.zeros((h, w), dtype=np.uint8)
                            master_obstacles[mask] = 1
        # Ensure obstacles array exists
        if master_obstacles is None:
            master_obstacles = np.zeros_like(heightmap, dtype=np.uint8)

        # Compute boundary margin and spacing based on map size if not already set
        h, w = heightmap.shape
        map_diag = math.sqrt(h ** 2 + w ** 2)
        constraints.boundary_margin_cells = max(1, int(min(h, w) * constraints.boundary_margin_ratio))
        constraints.min_spacing_cells = max(1.0, map_diag * constraints.min_spacing_ratio)
        constraints.max_spacing_cells = max(2.0, map_diag * constraints.max_spacing_ratio)

        # Return heightmap as a single‑element list to keep downstream expectations
        return [heightmap], master_obstacles, rover_cfg, constraints, []

# ====================================================
# STEP 2: POISSON DISK CANDIDATE SAMPLER
# ====================================================

class CandidateSampler:
    """Generates evenly spaced spatial candidates via Poisson‑disk sampling."""

    def __init__(self, map_shape: Tuple[int, int], boundary_margin: int, seed: Optional[int] = None):
        self.height, self.width = map_shape
        self.margin_y = max(0, min(boundary_margin, (self.height - 1) // 2))
        self.margin_x = max(0, min(boundary_margin, (self.width - 1) // 2))
        self.rng = np.random.default_rng(seed)

    def sample_poisson(self, min_dist: float, max_candidates: int) -> np.ndarray:
        min_dist = max(1.0, min(min_dist, min(self.width, self.height) / 2.0))
        cell_size = min_dist / math.sqrt(2)
        grid_w = max(1, int(math.ceil(self.width / cell_size)))
        grid_h = max(1, int(math.ceil(self.height / cell_size)))
        grid = np.full((grid_h, grid_w), -1, dtype=np.int32)
        candidates = []
        active = []
        low_x, high_x = self.margin_x, self.width - self.margin_x
        low_y, high_y = self.margin_y, self.height - self.margin_y
        init_x = int(self.rng.integers(low_x, high_x)) if high_x > low_x else self.width // 2
        init_y = int(self.rng.integers(low_y, high_y)) if high_y > low_y else self.height // 2
        first_pt = np.array([init_x, init_y], dtype=np.int32)
        candidates.append(first_pt)
        active.append(0)
        gx = min(grid_w - 1, int(init_x / cell_size))
        gy = min(grid_h - 1, int(init_y / cell_size))
        grid[gy, gx] = 0
        while active and len(candidates) < max_candidates:
            idx = self.rng.integers(0, len(active))
            cur_idx = active[idx]
            pt = candidates[cur_idx]
            found = False
            for _ in range(30):
                rad = self.rng.uniform(min_dist, 2 * min_dist)
                ang = self.rng.uniform(0, 2 * math.pi)
                new_x = int(pt[0] + rad * math.cos(ang))
                new_y = int(pt[1] + rad * math.sin(ang))
                if (self.margin_x <= new_x < self.width - self.margin_x and
                        self.margin_y <= new_y < self.height - self.margin_y):
                    gx = min(grid_w - 1, int(new_x / cell_size))
                    gy = min(grid_h - 1, int(new_y / cell_size))
                    conflict = False
                    for i in range(max(0, gy - 2), min(grid_h, gy + 3)):
                        for j in range(max(0, gx - 2), min(grid_w, gx + 3)):
                            neighbor = grid[i, j]
                            if neighbor != -1:
                                if np.linalg.norm(candidates[neighbor] - [new_x, new_y]) < min_dist:
                                    conflict = True
                                    break
                        if conflict:
                            break
                    if not conflict:
                        new_pt = np.array([new_x, new_y], dtype=np.int32)
                        candidates.append(new_pt)
                        c_idx = len(candidates) - 1
                        active.append(c_idx)
                        grid[gy, gx] = c_idx
                        found = True
                        break
            if not found:
                active.pop(idx)
        return np.array(candidates, dtype=np.int32)

# ====================================================
# STEP 3: CANDIDATE FILTER (OBSTACLE CLEARANCE ONLY)
# ====================================================

class CandidateFilter:
    """Filters candidates based on obstacle clearance and optional map mask."""

    def __init__(self, obstacles: np.ndarray, valid_mask: np.ndarray = None):
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
        in_bounds = (x >= 0) & (x < w) & (y >= 0) & (y < h)
        if self.valid_mask is not None:
            mask_valid = np.zeros_like(in_bounds, dtype=bool)
            mask_valid[in_bounds] = self.valid_mask[y[in_bounds], x[in_bounds]]
        else:
            mask_valid = in_bounds
        safe_x = np.clip(x, 0, w - 1)
        safe_y = np.clip(y, 0, h - 1)
        max_possible = float(np.max(self.clearance_map)) if self.clearance_map.size > 0 else 0.0
        effective = min(float(required_clearance_cells), max_possible)
        clearance_ok = self.clearance_map[safe_y, safe_x] >= effective
        final = mask_valid & clearance_ok
        if not np.any(final):
            logging.warning(
                f"Clearance requirement ({required_clearance_cells}) exceeds available map clearance ({max_possible:.1f}) or all candidates out of bounds."
            )
        return candidates[final]

# ====================================================
# STEP 4: INCREMENTAL WAYPOINT BUILDER (5‑point missions)
# ====================================================

class waypointBuilder:
    """Constructs missions with a fixed number of waypoints (default 5)."""

    def __init__(self, min_spacing: float, max_spacing: float, seed: Optional[int] = None,
                 max_turn_angle_deg: float = 80.0, min_start_end_dist: float = 0.0):
        self.min_spacing = min_spacing
        self.max_spacing = max_spacing
        self.rng = np.random.default_rng(seed)
        self.max_turn_angle_deg = max_turn_angle_deg
        self.min_start_end_dist = min_start_end_dist
        self._cos_limit = math.cos(math.radians(max_turn_angle_deg))

    def build_waypoints(self, candidates: np.ndarray, target_count: int, max_attempts: int, home_coord: tuple) -> np.ndarray:
        if len(candidates) < 4:
            logging.warning("Fewer than 4 candidates available. Cannot build missions.")
            return np.array([])
        missions = []
        num_cand = len(candidates)
        attempts = 0
        dead_ends = 0
        
        home_pt = np.array(home_coord, dtype=np.float32)
        
        while len(missions) < target_count and attempts < max_attempts:
            attempts += 1
            wp = [home_pt]
            visited = set()
            prev_vec = None
            success = True
            for _ in range(4):  # 4 intermediate points
                cur = wp[-1]
                dists = np.linalg.norm(candidates - cur, axis=1)
                valid = (dists >= self.min_spacing) & (dists <= self.max_spacing)
                idxs = np.where(valid)[0]
                idxs = [i for i in idxs if i not in visited]
                if prev_vec is not None and idxs:
                    vecs = candidates[idxs] - cur
                    norms = np.linalg.norm(vecs, axis=1)
                    cos_angles = (vecs @ prev_vec) / (norms * np.linalg.norm(prev_vec) + 1e-12)
                    idxs = [i for i, ca in zip(idxs, cos_angles) if ca >= self._cos_limit]
                if not idxs:
                    success = False
                    dead_ends += 1
                    break
                next_idx = self.rng.choice(idxs)
                visited.add(next_idx)
                next_pt = candidates[next_idx]
                prev_vec = next_pt - cur
                wp.append(next_pt)
            if success:
                wp.append(home_pt) # Return to home
                missions.append(wp)
        logging.info(
            f"build_waypoints: {attempts} attempts | {len(missions)} accepted | {dead_ends} dead‑ends"
        )
        return np.array(missions, dtype=np.int32)

# ====================================================
# STEP 5: WAYPOINT VALIDATOR (SELF‑INTERSECTION & SPACING)
# ====================================================

class waypointValidator:
    """Ensures missions respect non‑consecutive spacing constraints."""

    @staticmethod
    def validate(waypoints: np.ndarray, min_spacing: float) -> np.ndarray:
        if len(waypoints) == 0:
            return waypoints
        valid = []
        half = min_spacing * 0.5
        for wp in waypoints:
            ok = True
            for i in range(len(wp)):
                for j in range(i + 2, len(wp)):
                    if i == 0 and j == len(wp) - 1:
                        continue # Start and end (Home) are allowed to be the same
                    if np.linalg.norm(wp[i] - wp[j]) < half:
                        ok = False
                        break
                if not ok:
                    break
            if ok:
                valid.append(wp)
        return np.array(valid, dtype=np.int32)

# ====================================================
# STEP 6: SIMILARITY FILTER (DEDUPLICATION)
# ====================================================

class SimilarityFilter:
    """Removes near‑duplicate missions using spatial hashing."""

    @staticmethod
    def deduplicate(waypoints: np.ndarray, spatial_threshold: float) -> np.ndarray:
        if len(waypoints) == 0:
            return waypoints
        unique = []
        seen = set()
        for wp in waypoints:
            quant = tuple((int(pt[0] // spatial_threshold), int(pt[1] // spatial_threshold)) for pt in wp)
            if quant not in seen:
                seen.add(quant)
                unique.append(wp)
        return np.array(unique, dtype=np.int32)

# ====================================================
# STEP 7: EXPORTER (WAYPOINTS ONLY)
# ====================================================

class Exporter:
    """Writes waypoint arrays to .npy files and produces a minimal generation log."""

    @staticmethod
    def _make_serializable(val):
        if isinstance(val, dict):
            return {str(k): Exporter._make_serializable(v) for k, v in val.items()}
        if isinstance(val, (list, tuple)):
            return [Exporter._make_serializable(v) for v in val]
        if isinstance(val, (np.float32, np.float64, np.floating)):
            return float(val)
        if isinstance(val, (np.int32, np.int64, np.integer)):
            return int(val)
        if isinstance(val, np.ndarray):
            return val.tolist()
        return val

    @staticmethod
    def export(
        waypoints: np.ndarray,
        map_names: List[str],
        rover_cfg: RoverConfig,
        constraints: Constraints,
        outputs_dir: str = "outputs",
    ):
        out_path = Path(outputs_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        for old in out_path.glob("wp*_*.npy"):
            old.unlink()
        for i, wp in enumerate(waypoints):
            filename = f"wp{i:02d}.npy"
            np.save(out_path / filename, wp)
        metadata = {
            "generator_version": "2.1.0",
            "timestamp": datetime.now().isoformat(),
            "maps_processed": map_names,
            "wp_count": int(len(waypoints)),
            "rover_config": rover_cfg.__dict__,
            "constraints": constraints.__dict__,
        }
        with open(out_path / "generation_log.json", "w") as f:
            json.dump(Exporter._make_serializable(metadata), f, indent=4)
        logging.info(f"Exported {len(waypoints)} waypoint missions to '{out_path.resolve()}'")

# ====================================================
# PIPELINE ORCHESTRATOR
# ====================================================

def run_generator():
    start = time.time()
    logging.info("--- Starting Waypoint Generator Pipeline (No Difficulty Scoring) ---")
    heightmaps, master_obstacles, rover_cfg, constraints, map_names = InputLoader.load("inputs")
    valid_mask = _compute_valid_map_mask(heightmaps, constraints.boundary_margin_cells)
    sampler = CandidateSampler(master_obstacles.shape, constraints.boundary_margin_cells, constraints.seed)
    raw_candidates = sampler.sample_poisson(max(1.0, constraints.min_spacing_cells * 0.5), constraints.candidate_count)
    cand_filter = CandidateFilter(master_obstacles, valid_mask=valid_mask)
    max_clr = float(np.max(cand_filter.clearance_map)) if cand_filter.clearance_map.size > 0 else 0.0
    logging.info(
        f"Required clearance: {rover_cfg.clearance_radius_cells} cells | Max available: {max_clr:.1f} cells"
    )
    valid_candidates = cand_filter.filter(raw_candidates, rover_cfg.clearance_radius_cells)
    logging.info(
        f"Valid candidates: {len(valid_candidates)} | spacing [{constraints.min_spacing_cells:.1f}, {constraints.max_spacing_cells:.1f}]"
    )
    if len(valid_candidates) < 5:
        logging.error("Insufficient candidates after filtering. Aborting.")
        Exporter.export(np.array([]), map_names, rover_cfg, constraints, "outputs")
        return
    builder = waypointBuilder(
        constraints.min_spacing_cells,
        constraints.max_spacing_cells,
        constraints.seed,
        max_turn_angle_deg=constraints.max_turn_angle_deg,
        min_start_end_dist=constraints.min_start_end_dist,
    )
    
    # Calculate Home coordinate (Gazebo X=0, Y=0) based on map origin
    # Map origin for the expanded 1456m2 world is x=-18.431461, y=-7.687172, resolution=0.25
    origin_x, origin_y = -18.431461, -7.687172
    res = 0.25
    home_coord = ((0.0 - origin_x) / res, (0.0 - origin_y) / res)
    
    raw_missions = builder.build_waypoints(
        valid_candidates,
        constraints.target_mission_count,
        constraints.max_attempts,
        home_coord=home_coord,
    )
    validated = waypointValidator.validate(raw_missions, constraints.min_spacing_cells)
    unique = SimilarityFilter.deduplicate(validated, constraints.duplicate_distance_threshold)
    snapped = enforce_valid_waypoints(unique, valid_mask)
    final_missions = snapped[: constraints.target_mission_count]
    Exporter.export(final_missions, map_names, rover_cfg, constraints, "outputs")
    elapsed = time.time() - start
    logging.info(f"--- Pipeline finished in {elapsed:.3f}s ---")

if __name__ == "__main__":
    run_generator()