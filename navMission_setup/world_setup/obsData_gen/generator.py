#!/usr/bin/env python3
"""
ROAR Rock Generator — Heightmap-based obstacle placement.

Workflow:
  1. Loads a pre-baked terrain heightmap (.npz) from world_setup inputs.
  2. Samples (X, Y) candidates randomly within the terrain bounds.
  3. Queries the heightmap for the real ground Z at each candidate.
  4. Rejects candidates on flat/void areas (Z < min_terrain_height).
  5. Enforces minimum spacing between accepted rocks.
  6. Saves the final list with accurate ground-level Z coordinates and dimensions to .npy.

Rock length/width/height are computed dynamically from each rock's mesh
bounding box in rocks_ws/rock_N/meshes/rock_N.obj, rather than a hardcoded
table -- so adding, removing, or replacing rock meshes is picked up
automatically without touching this script.

Because rocks are placed at their true Z, there is NO need for a physics
air-drop or a 2-second physics settle in Gazebo — they are baked in place.
"""
import os
import math
import random
import argparse
from pathlib import Path

import numpy as np

# Fixed ArUco marker locations to avoid rock collisions
ARUCO_MARKERS = [
    (3.183, 8.012), (7.269, 9.482), (7.878, 17.583), (9.225, 22.389),
    (3.518, 23.990), (0.882, 16.870), (-3.944, 21.415), (-5.491, 16.334),
    (-7.695, 13.528), (-1.610, 12.602), (-7.715, 9.721), (-4.311, 4.442),
    (-5.720, 28.118), (-11.438, 5.230), (6.483, 1.102)
]

# ─────────────────────────────────────────────────────────────────────────────
# Heightmap Sampler
# ─────────────────────────────────────────────────────────────────────────────

class HeightmapSampler:
    """
    Loads a pre-baked terrain heightmap (.npz) and provides:
      - get_height(x, y)           → interpolated Z, or NaN if out-of-bounds
      - is_valid_terrain(x, y)     → True if height is above the terrain threshold
      - terrain_bounds()           → (x_min, x_max, y_min, y_max) from the map

    Uses bilinear interpolation for smooth, accurate height queries.
    """

    def __init__(self, npz_path: str, min_terrain_height: float = 0.15,
                 min_roughness: float = 0.02):
        if not os.path.isfile(npz_path):
            raise FileNotFoundError(
                f"Heightmap not found: {npz_path}\n"
                "Ensure a valid heightmap .npz exists in inputs or initial_inputs."
            )
        with np.load(npz_path) as data:
            self.xs = data["xs"].astype(np.float64)   # shape (cols,)
            self.ys = data["ys"].astype(np.float64)   # shape (rows,)
            self.grid = data["grid"].astype(np.float64)  # shape (rows, cols)

        self.min_terrain_height = min_terrain_height
        self.min_roughness = min_roughness
        self.x_min = float(self.xs[0])
        self.x_max = float(self.xs[-1])
        self.y_min = float(self.ys[0])
        self.y_max = float(self.ys[-1])

        # Precompute local roughness grid (3x3 window std-dev)
        self._roughness = self._compute_roughness()

        print(f"[HeightmapSampler] Loaded: {npz_path}")
        print(f"  Grid size : {self.grid.shape[1]}x{self.grid.shape[0]} cells")
        print(f"  X range   : {self.x_min:.2f} → {self.x_max:.2f} m")
        print(f"  Y range   : {self.y_min:.2f} → {self.y_max:.2f} m")
        print(f"  Z range   : {np.nanmin(self.grid):.3f} → {np.nanmax(self.grid):.3f} m")
        print(f"  Min valid Z: {self.min_terrain_height:.3f} m")

        valid_cells = np.sum(~np.isnan(self.grid) & (self.grid >= min_terrain_height)
                             & (self._roughness >= min_roughness))
        total_cells = self.grid.size
        print(f"  Valid terrain cells: {valid_cells}/{total_cells} "
              f"({100.0 * valid_cells / total_cells:.1f}%) "
              f"[height>={min_terrain_height}m AND roughness>={min_roughness}m]")

    def get_height(self, x: float, y: float) -> float:
        """
        Bilinear interpolation of terrain Z at world coordinates (x, y).
        Returns NaN if the point is outside the heightmap extent.
        """
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            return float("nan")

        col_f = (x - self.x_min) / (self.x_max - self.x_min) * (len(self.xs) - 1)
        row_f = (y - self.y_min) / (self.y_max - self.y_min) * (len(self.ys) - 1)

        col0 = int(math.floor(col_f))
        row0 = int(math.floor(row_f))
        col1 = min(col0 + 1, len(self.xs) - 1)
        row1 = min(row0 + 1, len(self.ys) - 1)

        dc = col_f - col0
        dr = row_f - row0

        z00 = self.grid[row0, col0]
        z01 = self.grid[row0, col1]
        z10 = self.grid[row1, col0]
        z11 = self.grid[row1, col1]

        vals = [v for v in (z00, z01, z10, z11) if not math.isnan(v)]
        if not vals:
            return float("nan")
        if any(math.isnan(v) for v in (z00, z01, z10, z11)):
            return float(min(vals, key=lambda v: abs(v)))

        z = (z00 * (1 - dc) * (1 - dr) +
             z01 * dc * (1 - dr) +
             z10 * (1 - dc) * dr +
             z11 * dc * dr)
        return float(z)

    def _compute_roughness(self) -> np.ndarray:
        """Precomputes local Z standard deviation using a 3x3 sliding window."""
        rows, cols = self.grid.shape
        roughness = np.full_like(self.grid, np.nan)
        for r in range(rows):
            for c in range(cols):
                if math.isnan(self.grid[r, c]):
                    continue
                r0, r1 = max(0, r - 1), min(rows - 1, r + 1)
                c0, c1 = max(0, c - 1), min(cols - 1, c + 1)
                patch = self.grid[r0:r1 + 1, c0:c1 + 1]
                vals = patch[~np.isnan(patch)]
                if len(vals) >= 2:
                    roughness[r, c] = float(np.std(vals))
                else:
                    roughness[r, c] = 0.0
        return roughness

    def get_roughness(self, x: float, y: float) -> float:
        """Returns the precomputed local roughness (std-dev) at world (x, y)."""
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            return float("nan")
        col_f = (x - self.x_min) / (self.x_max - self.x_min) * (len(self.xs) - 1)
        row_f = (y - self.y_min) / (self.y_max - self.y_min) * (len(self.ys) - 1)
        col0 = max(0, min(len(self.xs) - 1, int(round(col_f))))
        row0 = max(0, min(len(self.ys) - 1, int(round(row_f))))
        return float(self._roughness[row0, col0])

    def is_valid_terrain(self, x: float, y: float) -> bool:
        """Returns True if (x, y) sits on actual rough terrain."""
        z = self.get_height(x, y)
        if math.isnan(z) or z < self.min_terrain_height:
            return False
        roughness = self.get_roughness(x, y)
        if math.isnan(roughness) or roughness < self.min_roughness:
            return False
        return True

    def terrain_bounds(self):
        """Returns (x_min, x_max, y_min, y_max) derived from the heightmap."""
        return self.x_min, self.x_max, self.y_min, self.y_max


# ─────────────────────────────────────────────────────────────────────────────
# Rock Mesh Dimensions (dynamic, read from rocks_ws meshes)
# ─────────────────────────────────────────────────────────────────────────────

# In-memory cache: {rocks_dir_str: {rock_id: {"length":..,"width":..,"height":..}}}
# Populated once per rocks_dir the first time it's needed, so a run placing
# hundreds of rock instances never re-parses the same .obj more than once.
_ROCK_DIM_CACHE: dict[str, dict[int, dict[str, float]]] = {}
_ROCK_VERTEX_CACHE: dict[str, np.ndarray] = {}


def _read_obj_extents(obj_path: Path) -> dict:
    """
    Parses vertex lines ('v x y z ...') out of a Wavefront .obj file and
    returns the axis-aligned bounding box size and limits in the mesh's own X/Y/Z coordinate frame.
    """
    min_v = [math.inf, math.inf, math.inf]
    max_v = [-math.inf, -math.inf, -math.inf]

    with open(obj_path, "r", errors="ignore") as f:
        for line in f:
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            for i, val in enumerate((x, y, z)):
                if val < min_v[i]:
                    min_v[i] = val
                if val > max_v[i]:
                    max_v[i] = val

    if any(math.isinf(v) for v in min_v):
        raise ValueError(f"No vertex data found in {obj_path}")

    return {
        "length": max_v[0] - min_v[0],
        "width": max_v[1] - min_v[1],
        "height": max_v[2] - min_v[2],
        "min_x": min_v[0], "max_x": max_v[0],
        "min_y": min_v[1], "max_y": max_v[1],
        "min_z": min_v[2], "max_z": max_v[2]
    }


def _read_obj_vertices(obj_path: Path) -> np.ndarray:
    """Return mesh vertices, cached for terrain-support Z calculations."""
    key = str(obj_path.resolve())
    cached = _ROCK_VERTEX_CACHE.get(key)
    if cached is not None:
        return cached
    vertices = []
    with open(obj_path, "r", errors="ignore") as mesh_file:
        for line in mesh_file:
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
    if not vertices:
        raise ValueError(f"No vertex data found in {obj_path}")
    result = np.asarray(vertices, dtype=np.float64)
    _ROCK_VERTEX_CACHE[key] = result
    return result


def calculate_supported_rock_z(
    sampler: HeightmapSampler,
    rocks_dir: Path,
    mesh_id: int,
    x: float,
    y: float,
    yaw: float,
) -> float:
    """Place a yawed mesh on the collision surface without footprint burial.

    The required model-origin Z is the maximum ``terrain_z - vertex_z`` over
    the yawed mesh footprint.  Thus at least one mesh vertex contacts the
    terrain and no sampled mesh vertex is below it.  Roll/pitch remain exactly
    zero, preserving the generator's existing orientation rule.
    """
    obj_path = rocks_dir / f"rock_{mesh_id}" / "meshes" / f"rock_{mesh_id}.obj"
    vertices = _read_obj_vertices(obj_path)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    requirements = []
    for local_x, local_y, local_z in vertices:
        world_x = x + cosine * local_x - sine * local_y
        world_y = y + sine * local_x + cosine * local_y
        terrain_z = sampler.get_height(world_x, world_y)
        if not math.isnan(terrain_z):
            requirements.append(terrain_z - local_z)
    if not requirements:
        raise RuntimeError(
            f"No terrain samples under rock_{mesh_id} at ({x:.3f}, {y:.3f})"
        )
    return float(max(requirements))


def discover_rock_ids(rocks_dir: Path) -> list[int]:
    """
    Scans rocks_dir for rock_N/meshes/rock_N.obj folders and returns the
    sorted list of valid integer rock IDs found. A rock_N folder without a
    matching rock_N.obj mesh is skipped with a warning.
    """
    ids = []
    for entry in sorted(rocks_dir.glob("rock_*")):
        if not entry.is_dir():
            continue
        suffix = entry.name.split("_", 1)[-1]
        if not suffix.isdigit():
            continue
        rock_id = int(suffix)
        obj_path = entry / "meshes" / f"rock_{rock_id}.obj"
        if obj_path.is_file():
            ids.append(rock_id)
        else:
            print(f"  [Warning] {entry.name}: no meshes/rock_{rock_id}.obj found — skipping.")
    return sorted(ids)


def get_rock_dimensions(rocks_dir: Path) -> dict[int, dict[str, float]]:
    """
    Returns {rock_id: {"length","width","height"}} for every rock mesh found
    in rocks_dir, computing bounding boxes on first use and caching per
    rocks_dir for the remainder of this process.
    """
    key = str(rocks_dir.resolve())
    if key in _ROCK_DIM_CACHE:
        return _ROCK_DIM_CACHE[key]

    dims: dict[int, dict[str, float]] = {}
    for rock_id in discover_rock_ids(rocks_dir):
        obj_path = rocks_dir / f"rock_{rock_id}" / "meshes" / f"rock_{rock_id}.obj"
        try:
            dims[rock_id] = _read_obj_extents(obj_path)
        except Exception as exc:
            print(f"  [Warning] Could not read mesh for rock_{rock_id} ({obj_path}): {exc}")
            continue

    _ROCK_DIM_CACHE[key] = dims
    return dims


def classify_rock_sizes(
    rock_dimensions: dict[int, dict[str, float]],
    min_collidable_size_m: float = 0.15,
) -> tuple[list[int], list[int]]:
    """
    Splits rock IDs into (collidable_ids, non_collidable_ids) based on their
    measured mesh height -- a rock is considered collidable if its bounding
    box height meets or exceeds min_collidable_size_m.

    This replaces any assumption about folder naming/ordering (e.g. "first N
    rocks are big") with an actual size measurement, so it keeps working
    correctly even if rocks_ws is reordered, renamed, or gains new meshes.
    """
    collidable_ids = []
    non_collidable_ids = []
    for rock_id, dims in rock_dimensions.items():
        if dims["height"] >= min_collidable_size_m:
            collidable_ids.append(rock_id)
        else:
            non_collidable_ids.append(rock_id)
    return sorted(collidable_ids), sorted(non_collidable_ids)


def build_model_id_map(
    collidable_mesh_ids: list,
    non_collidable_mesh_ids: list,
    balance_pools: bool = True,
) -> dict:
    """
    Builds the canonical 'rock_id' -> mesh mapping that gets exported and
    consumed downstream (including the mission-scoring package that
    blacklists non-collidable rocks). This is a SEPARATE id space from the
    rocks_ws folder numbers (mesh_id) -- rock_id is what other packages
    should read, mesh_id only says which .obj to render for that rock_id.

    Convention:  ODD rock_id  -> collidable
                 EVEN rock_id -> non-collidable
    So collidability can be checked with a single test, no lookup table
    required downstream: `is_collidable = (rock_id % 2 == 1)`.

    If balance_pools is True and the collidable/non-collidable mesh counts
    differ, the smaller pool's meshes are cycled/reused until both pools are
    the same length, so the odd and even id ranges come out equal-sized
    instead of one side being a single mesh reused far more than the other.
    """
    collidable = list(collidable_mesh_ids)
    non_collidable = list(non_collidable_mesh_ids)

    if balance_pools and collidable and non_collidable:
        target = max(len(collidable), len(non_collidable))
        if len(collidable) < target:
            collidable = [collidable[i % len(collidable_mesh_ids)] for i in range(target)]
        if len(non_collidable) < target:
            non_collidable = [non_collidable[i % len(non_collidable_mesh_ids)] for i in range(target)]

    model_map = {}
    for i, mesh_id in enumerate(collidable):
        rock_id = 2 * i + 1  # 1, 3, 5, ...
        model_map[rock_id] = {"mesh_id": mesh_id, "is_collidable": True}
    for i, mesh_id in enumerate(non_collidable):
        rock_id = 2 * i + 2  # 2, 4, 6, ...
        model_map[rock_id] = {"mesh_id": mesh_id, "is_collidable": False}
    return model_map


def get_default_rocks_dir() -> Path | None:
    """Locate rocks_ws relative to this script (world_setup/rocks_ws)."""
    current_dir = Path(__file__).resolve().parent
    world_setup_dir = current_dir.parent
    candidate = world_setup_dir / "rocks_ws"
    return candidate if candidate.is_dir() else None


def get_heightmap_path():
    """Locate marsyard_heightmap.npz inside navMission_setup directory structure."""
    current_dir = Path(__file__).resolve().parent
    world_setup_dir = current_dir.parent

    candidates = [
        current_dir / "inputs" / "marsyard_heightmap.npz",
        world_setup_dir / "initial_inputs" / "i_heightmap" / "marsyard_heightmap.npz",
    ]

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)

    return None


# ─────────────────────────────────────────────────────────────────────────────
# Main generation function
# ─────────────────────────────────────────────────────────────────────────────

def generate_obstacle_data(
    world_name="marsyard.world",
    density=0.012,
    collidable_ratio=0.5,
    spacing=1.0,
    x_range=(-19.0, 13.0),
    y_range=(-11.0, 11.0),
    min_terrain_height=0.15,
    min_roughness=0.02,
    deadends=False,
    output_file=None,
    heightmap_path=None,
    rocks_dir=None,
    min_collidable_size_m=0.15,
    balance_model_pools=True,
    clean_previous_outputs=True,
):
    """
    Generates rock obstacle configuration using the terrain heightmap and saves
    as a numpy (.npy) file matching the worldData_example schema.
    """
    if heightmap_path is None:
        heightmap_path = get_heightmap_path()

    if heightmap_path is None or not os.path.isfile(heightmap_path):
        raise FileNotFoundError(
            f"Could not locate valid heightmap file: {heightmap_path}\n"
            "Provide a valid .npz file via --heightmap parameter."
        )

    if rocks_dir is None:
        rocks_dir = get_default_rocks_dir()
    if rocks_dir is None:
        raise FileNotFoundError(
            "Could not locate rocks_ws directory. Provide a valid path via --rocks-dir."
        )
    rocks_dir = Path(rocks_dir).resolve()
    if not rocks_dir.is_dir():
        raise FileNotFoundError(f"rocks_dir does not exist or is not a directory: {rocks_dir}")

    rock_dimensions = get_rock_dimensions(rocks_dir)
    if not rock_dimensions:
        raise RuntimeError(
            f"No valid rock meshes found under {rocks_dir} "
            "(expected rocks_dir/rock_N/meshes/rock_N.obj)."
        )
    available_rock_ids = sorted(rock_dimensions.keys())
    collidable_rock_ids, non_collidable_rock_ids = classify_rock_sizes(
        rock_dimensions, min_collidable_size_m
    )
    if not collidable_rock_ids:
        print(f"  [Warning] No rock mesh reaches min_collidable_size_m={min_collidable_size_m}m "
              "-- all rocks will be non-collidable.")
    if not non_collidable_rock_ids:
        print(f"  [Warning] Every rock mesh meets/exceeds min_collidable_size_m={min_collidable_size_m}m "
              "-- all rocks will be collidable.")

    model_map = build_model_id_map(
        collidable_rock_ids, non_collidable_rock_ids, balance_pools=balance_model_pools
    )
    collidable_model_ids = sorted(rid for rid, info in model_map.items() if info["is_collidable"])
    non_collidable_model_ids = sorted(rid for rid, info in model_map.items() if not info["is_collidable"])

    sampler = HeightmapSampler(
        heightmap_path,
        min_terrain_height=min_terrain_height,
        min_roughness=min_roughness,
    )

    x_min, x_max, y_min, y_max = sampler.terrain_bounds()
    x_min += 0.5
    x_max -= 0.5
    y_min += 0.5
    y_max -= 0.5

    if output_file is None:
        script_dir = Path(__file__).resolve().parent
        output_dir = script_dir / "outputs"
        output_dir.mkdir(parents=True, exist_ok=True)
        output_file = str(output_dir / "obstacle_data.npy")
    else:
        output_dir = Path(output_file).resolve().parent
        output_dir.mkdir(parents=True, exist_ok=True)

    if clean_previous_outputs:
        removed = 0
        for old in list(output_dir.glob("obstacle_data*.npy")) + list(output_dir.glob("*_info.txt")):
            old.unlink()
            removed += 1
        if removed:
            print(f"  Cleared {removed} previous output file(s) from {output_dir}")

    output_dir_str = str(output_dir)

    area = (x_max - x_min) * (y_max - y_min)
    num_rocks = max(1, int(round(density * area)))

    print("=" * 60)
    print("   Generating Obstacle Data File (.npy) — Heightmap Mode")
    print("=" * 60)
    print(f"  Target World         : {world_name}")
    print(f"  Heightmap            : {heightmap_path}")
    print(f"  Rocks Dir            : {rocks_dir}")
    print(f"  Rock IDs Discovered  : {available_rock_ids}")
    for rid in available_rock_ids:
        d = rock_dimensions[rid]
        tag = "collidable" if rid in collidable_rock_ids else "non-collidable"
        print(f"    rock_{rid}: L={d['length']:.3f} m  W={d['width']:.3f} m  H={d['height']:.3f} m  [{tag}]")
    print(f"  Min Collidable Height: {min_collidable_size_m:.3f} m")
    print(f"  Rock ID Map (odd=collidable, even=non-collidable) -- this is the id downstream packages should read:")
    for rid in sorted(model_map.keys()):
        info = model_map[rid]
        d = rock_dimensions[info["mesh_id"]]
        tag = "collidable" if info["is_collidable"] else "non-collidable"
        print(f"    rock_id={rid:>2} [{tag:<14}] -> mesh=rock_{info['mesh_id']}  "
              f"L={d['length']:.3f} m  W={d['width']:.3f} m  H={d['height']:.3f} m")
    print(f"  Terrain Bounds (sampling): X=[{x_min:.1f}, {x_max:.1f}]  Y=[{y_min:.1f}, {y_max:.1f}]")
    print(f"  Calculated Area      : {area:.1f} m²")
    print(f"  Rock Density         : {density} rocks/m²")
    print(f"  Total Rocks (target) : {num_rocks}")
    print(f"  Collidable Ratio     : {collidable_ratio:.2f}")
    print(f"  Min Spacing          : {spacing:.2f} m")
    print(f"  Min Terrain Height   : {min_terrain_height:.3f} m")
    print(f"  Min Roughness        : {min_roughness:.4f} m (rejects flat areas)")
    print(f"  Deadends             : {deadends}")
    print(f"  Output File          : {output_file}")
    print("=" * 60)

    placed_coords = []
    rock_data_list = []

    # Barrier rocks (deadends)
    if deadends:
        barrier_xs = [-3.0, -1.5, 0.0, 1.5, 3.0]
        num_barrier = min(num_rocks, len(barrier_xs))
        # Barriers must actually block the rover, so always draw from the
        # collidable (odd rock_id) pool; fall back to any rock if none qualify.
        barrier_pool = collidable_model_ids or list(model_map.keys())
        for idx, bx in enumerate(barrier_xs[:num_barrier], 1):
            by = 2.5
            chosen_model_id = random.choice(barrier_pool)
            mesh_id = model_map[chosen_model_id]["mesh_id"]
            yaw = float(random.uniform(-math.pi, math.pi))
            supported_z = calculate_supported_rock_z(
                sampler, rocks_dir, int(mesh_id), float(bx), float(by), yaw
            )

            rock_entry = {
                "id": idx,
                "name": f"Rock_{idx}",
                "x": float(bx),
                "y": float(by),
                "z": supported_z,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": yaw,
                "rock_id": int(chosen_model_id),
                "mesh_id": int(mesh_id),
                "is_collidable": True,
                "is_barrier": True,
                "world_name": str(world_name),
            }
            placed_coords.append((bx, by))
            rock_data_list.append(rock_entry)

    # Random rocks
    start_idx = len(rock_data_list) + 1
    for idx in range(start_idx, num_rocks + 1):
        best_x = best_y = None
        max_attempts = 1000

        for _ in range(max_attempts):
            cand_x = random.uniform(x_min, x_max)
            cand_y = random.uniform(y_min, y_max)

            if not sampler.is_valid_terrain(cand_x, cand_y):
                continue

            # Original exclusion zone
            if (7.0 <= cand_x <= 14.0) and (-5.0 <= cand_y <= 5.0):
                continue

            # Prevent rocks from spawning exactly where the rover drops in (0, 0, 2.5)
            if (-2.5 <= cand_x <= 2.5) and (-2.5 <= cand_y <= 2.5):
                continue

            # Prevent rocks from spawning exactly where the rover drops in (0, 0, 2.5)
            if (-2.5 <= cand_x <= 2.5) and (-2.5 <= cand_y <= 2.5):
                continue

            # Prevent rocks from spawning on top of ArUco markers (1.0m exclusion radius)
            too_close_to_aruco = any(
                math.hypot(cand_x - ax, cand_y - ay) < 1.0
                for ax, ay in ARUCO_MARKERS
            )
            if too_close_to_aruco:
                continue

            too_close = any(
                math.hypot(cand_x - px, cand_y - py) < spacing
                for px, py in placed_coords
            )
            if too_close:
                continue

            best_x, best_y = cand_x, cand_y
            break

        if best_x is None:
            print(
                f"  [Warning] Rock {idx}: could not place with spacing={spacing:.2f}m "
                f"after {max_attempts} attempts — skipping."
            )
            continue

        # collidable_ratio controls the probability of drawing from the
        # collidable (odd rock_id) pool vs the non-collidable (even rock_id)
        # pool. rock_id is the field downstream packages should read --
        # collidability is fully determined by its parity, no lookup needed.
        if collidable_model_ids and (not non_collidable_model_ids or random.random() < collidable_ratio):
            chosen_model_id = random.choice(collidable_model_ids)
        elif non_collidable_model_ids:
            chosen_model_id = random.choice(non_collidable_model_ids)
        else:
            chosen_model_id = random.choice(list(model_map.keys()))

        mesh_id = model_map[chosen_model_id]["mesh_id"]
        is_collidable = model_map[chosen_model_id]["is_collidable"]
        yaw = float(random.uniform(-math.pi, math.pi))
        supported_z = calculate_supported_rock_z(
            sampler, rocks_dir, int(mesh_id), float(best_x), float(best_y), yaw
        )

        rock_entry = {
            "id": idx,
            "name": f"Rock_{idx}",
            "x": float(best_x),
            "y": float(best_y),
            "z": supported_z,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": yaw,
            "rock_id": int(chosen_model_id),
            "mesh_id": int(mesh_id),
            "is_collidable": is_collidable,
            "is_barrier": False,
            "world_name": str(world_name),
        }
        placed_coords.append((best_x, best_y))
        rock_data_list.append(rock_entry)

    placed = len(rock_data_list)
    print(f"\n  Placed {placed}/{num_rocks} rocks successfully.")

    # Format schema matching worldData_example
    obs_array = np.array(rock_data_list, dtype=object)
    for rock_item in obs_array:
        rock = rock_item.item() if hasattr(rock_item, "item") else rock_item
        rock["frame_id"] = "world"
        dimensions = rock_dimensions[int(rock["mesh_id"])]
        rock["length"] = float(dimensions["length"])
        rock["width"] = float(dimensions["width"])
        rock["height"] = float(dimensions["height"])

    np.save(output_file, obs_array)
    print(f"  -> Saved: {output_file}")

    info_file = os.path.splitext(output_file)[0] + "_info.txt"
    with open(info_file, "w") as f:
        f.write("Obstacle Data Summary\n")
        f.write(f"World Name      : {world_name}\n")
        f.write(f"Heightmap       : {heightmap_path}\n")
        f.write(f"Rocks Dir       : {rocks_dir}\n")
        f.write(f"Total Placed    : {placed}\n")
        f.write(f"Density         : {density} rocks/m²\n")
        f.write(f"Collidable Ratio: {collidable_ratio}\n")
        f.write(f"Spacing Min     : {spacing} m\n")
        f.write(f"Min Terrain Z   : {min_terrain_height} m\n")
        f.write(f"Deadends        : {deadends}\n")
        f.write(f"Min Collidable Height: {min_collidable_size_m} m\n\n")
        f.write("Rock ID Map (odd=collidable, even=non-collidable) -- read this for blacklisting:\n")
        for rid in sorted(model_map.keys()):
            info = model_map[rid]
            d = rock_dimensions[info["mesh_id"]]
            tag = "collidable" if info["is_collidable"] else "non-collidable"
            f.write(f"  rock_id={rid:>2} [{tag:<14}] -> mesh=rock_{info['mesh_id']}  "
                    f"L={d['length']:.3f} m  W={d['width']:.3f} m  H={d['height']:.3f} m\n")
        f.write("\n")
        for r in rock_data_list:
            f.write(
                f"[{r['id']:3d}] {r['name']:<12} | rock_id={r['rock_id']:>2} "
                f"({'collidable' if r['is_collidable'] else 'non-collidable'}) | "
                f"Pos: ({r['x']:7.3f}, {r['y']:7.3f}, {r['z']:6.3f}) | yaw={r['yaw']:6.3f}\n"
            )
        f.write("\n(mesh + dimensions for each rock_id are in the Rock ID Map above, not repeated per-instance)\n")

    print("=" * 60)
    return output_file


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate Obstacle Data (.npy) for ROAR Simulation (Heightmap mode)"
    )
    parser.add_argument(
        "--world-name", type=str, default="marsyard.world",
        help="Target world name (default: marsyard.world)",
    )
    parser.add_argument(
        "--density", type=float, default=0.012,
        help="Rock density in rocks/m² (default: 0.012)",
    )
    parser.add_argument(
        "-c", "--collidable-ratio", type=float, default=0.5,
        help="Probability of drawing a placed rock from the collidable (large) mesh "
             "pool vs the non-collidable (small) pool (default: 0.5)",
    )
    parser.add_argument(
        "--min-collidable-height", dest="min_collidable_size", type=float, default=0.15,
        help="A rock mesh is classified collidable if its measured mesh height "
             "meets or exceeds this, in metres (default: 0.15)",
    )
    parser.add_argument(
        "-s", "--spacing", type=float, default=1.0,
        help="Minimum centre-to-centre spacing between rocks in metres (default: 1.0)",
    )
    parser.add_argument(
        "--min-roughness", type=float, default=0.02,
        help="Min local Z std-dev to accept a cell as rough terrain (default: 0.02).",
    )
    parser.add_argument(
        "--min-terrain-height", type=float, default=0.15,
        help="Minimum Z to consider valid terrain (default: 0.15)",
    )
    parser.add_argument(
        "--deadends", action="store_true", default=False,
        help="Place a barrier formation of rocks across the course centre",
    )
    parser.add_argument(
        "--heightmap", type=str, default=None,
        help="Override path to heightmap .npz",
    )
    parser.add_argument(
        "--rocks-dir", type=str, default=None,
        help="Override path to rocks_ws directory (default: world_setup/rocks_ws)",
    )
    parser.add_argument(
        "--no-balance-model-pools", dest="balance_model_pools", action="store_false", default=True,
        help="Disable reusing meshes to equalize collidable/non-collidable pool sizes "
             "(by default the smaller pool is cycled so odd/even rock_id ranges match in count).",
    )
    parser.add_argument(
        "--no-clean-outputs", dest="clean_previous_outputs", action="store_false", default=True,
        help="Keep old obstacle_data*.npy / *_info.txt files instead of deleting them before this run.",
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Output .npy path",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    generate_obstacle_data(
        world_name=args.world_name,
        density=args.density,
        collidable_ratio=args.collidable_ratio,
        spacing=args.spacing,
        min_terrain_height=args.min_terrain_height,
        min_roughness=args.min_roughness,
        deadends=args.deadends,
        output_file=args.output,
        heightmap_path=args.heightmap,
        rocks_dir=args.rocks_dir,
        min_collidable_size_m=args.min_collidable_size,
        balance_model_pools=args.balance_model_pools,
        clean_previous_outputs=args.clean_previous_outputs,
    )


if __name__ == "__main__":
    main()
