#!/usr/bin/env python3
"""
ROAR Rock Generator — Heightmap-based obstacle placement.

Workflow:
  1. Loads a pre-baked terrain heightmap (.npz) produced by
     rock_generator/maps_tools/heightmap/heightmap_generator.py.
  2. Samples (X, Y) candidates randomly within the terrain bounds.
  3. Queries the heightmap for the real ground Z at each candidate.
  4. Rejects candidates on flat/void areas (Z < min_terrain_height).
  5. Enforces minimum spacing between accepted rocks.
  6. Saves the final list with accurate ground-level Z coordinates to .npy.

Because rocks are placed at their true Z, there is NO need for a physics
air-drop or a 2-second physics settle in Gazebo — they are baked in place.
"""
import os
import math
import random
import argparse
import datetime

import numpy as np

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
        """
        Args:
            npz_path: Absolute path to the heightmap .npz file.
            min_terrain_height: Z threshold below which a cell is treated as
                                 void/flat-ground and rejected. Default 0.15 m
                                 matches the marsyard terrain floor.
            min_roughness: Minimum local Z standard deviation (metres) for a
                           cell to be accepted. Flat platform areas have
                           roughness ~0.001; rocky terrain has roughness >0.03.
                           Default 0.02 m safely rejects the flat grid.
        """
        if not os.path.isfile(npz_path):
            raise FileNotFoundError(
                f"Heightmap not found: {npz_path}\n"
                "Run rock_generator/maps_tools/heightmap/heightmap_generator.py first to generate it."
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

        # Fractional indices
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

        # If any neighbour is NaN, fall back to nearest-valid
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
        """Precomputes local Z standard deviation using a 3x3 sliding window.
        Flat areas → near zero; rocky terrain → high values.
        NaN cells produce NaN roughness (also rejected)."""
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
        """Returns True if (x, y) sits on actual rough terrain.
        Rejects: NaN voids, flat platform (low roughness), and below-threshold Z.
        """
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
# Package path helpers
# ─────────────────────────────────────────────────────────────────────────────

def get_package_src_dir():
    """Locates the source directory of the rock_generator package inside src/."""
    # Prioritize correct source workspace directory structure
    hardcoded = '/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/rock_generator'
    if os.path.exists(os.path.join(hardcoded, 'package.xml')):
        return hardcoded

    mod_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_src = os.path.dirname(mod_dir)
    if os.path.exists(os.path.join(pkg_src, "package.xml")):
        return pkg_src

    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("rock_generator")
        parts = share_dir.split(os.sep)
        if "install" in parts:
            idx = parts.index("install")
            ws_root = os.sep.join(parts[:idx])
            target_src = os.path.join(
                ws_root, "src", "navMission_setup", "rock_generator"
            )
            if os.path.exists(os.path.join(target_src, "package.xml")):
                return target_src
    except Exception:
        pass

    return pkg_src



def get_heightmap_path():
    """Locate marsyard_heightmap.npz dynamically inside the package."""
    pkg_src = get_package_src_dir()
    candidate = os.path.join(pkg_src, "rock_generator", "maps_tools", "heightmap", "data", "marsyard_heightmap.npz")
    if os.path.isfile(candidate):
        return candidate

    # Fallback: try install share dir
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("rock_generator")
        candidate = os.path.join(share, "rock_generator", "maps_tools", "heightmap", "data", "marsyard_heightmap.npz")
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

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
):
    """
    Generates rock obstacle configuration using the terrain heightmap and saves
    as a numpy (.npy) file directly inside the src/ package folder.

    Args:
        world_name:           Target Gazebo world name.
        x_range:              (min, max) X sampling boundary in metres.
        y_range:              (min, max) Y sampling boundary in metres.
        density:              Rocks per square metre of the sampling bounding box.
        collidable_ratio:     Fraction of rocks that have physics collisions.
        spacing:              Minimum centre-to-centre distance between rocks (m).
        min_terrain_height:   Minimum Z value a point must have to be considered
                              real terrain (rejects flat void areas).
        min_roughness:        Minimum local Z std-dev to reject flat surfaces.
        deadends:             If True, place a barrier of rocks across the centre.
        output_file:          Override the output .npy path.
        heightmap_path:       Override the heightmap .npz path.
    """
    pkg_src_dir = get_package_src_dir()

    # ── 1. Load Heightmap ────────────────────────────────────────────────────
    if heightmap_path is None:
        heightmap_path = get_heightmap_path()

    if heightmap_path is None:
        raise FileNotFoundError(
            "Could not find marsyard_heightmap.npz. "
            "Run rock_generator/maps_tools/heightmap/heightmap_generator.py first."
        )

    sampler = HeightmapSampler(heightmap_path, min_terrain_height=min_terrain_height,
                               min_roughness=min_roughness)

    # Use user-supplied bounds as the sampling window (NOT the full heightmap extent)
    x_min, x_max = x_range
    y_min, y_max = y_range

    # ── 2. Output paths ──────────────────────────────────────────────────────
    if output_file is None:
        output_dir = os.path.join(pkg_src_dir, "obs_data")
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, "obstacle_data.npy")
    else:
        output_dir = os.path.dirname(os.path.abspath(output_file))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

    # ── 3. Calculate rock count ──────────────────────────────────────────────
    area = (x_max - x_min) * (y_max - y_min)
    num_rocks = max(1, int(round(density * area)))

    print("=" * 60)
    print("   Generating Obstacle Data File (.npy) — Heightmap Mode")
    print("=" * 60)
    print(f"  Target World         : {world_name}")
    print(f"  Heightmap            : {heightmap_path}")
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

    # ── 4. Build rock list ───────────────────────────────────────────────────
    placed_coords = []
    rock_data_list = []

    # Barrier rocks (deadends)
    if deadends:
        barrier_xs = [-3.0, -1.5, 0.0, 1.5, 3.0]
        num_barrier = min(num_rocks, len(barrier_xs))
        for idx, bx in enumerate(barrier_xs[:num_barrier], 1):
            by = 2.5
            z = sampler.get_height(bx, by)
            if math.isnan(z):
                z = min_terrain_height  # safe fallback
            rock_entry = {
                "id": idx,
                "name": f"Rock_{idx}",
                "x": float(bx),
                "y": float(by),
                "z": float(z),
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": float(random.uniform(-math.pi, math.pi)),
                "rock_id": int(random.randint(1, 5)),
                "is_collidable": True,
                "is_barrier": True,
                "world_name": str(world_name),
            }
            placed_coords.append((bx, by))
            rock_data_list.append(rock_entry)

    # Random rocks
    start_idx = len(rock_data_list) + 1
    for idx in range(start_idx, num_rocks + 1):
        best_x = best_y = best_z = None
        max_attempts = 1000

        for _ in range(max_attempts):
            cand_x = random.uniform(x_min, x_max)
            cand_y = random.uniform(y_min, y_max)

            # ── Terrain validity check (heightmap Z + roughness)
            if not sampler.is_valid_terrain(cand_x, cand_y):
                continue

            # ── Exclusion Zone: flat white platform (measured from Gazebo)
            # This area is part of the collision mesh but has no visual terrain.
            if (7.0 <= cand_x <= 14.0) and (-11.0 <= cand_y <= 1.0):
                continue

            # ── Minimum spacing check
            too_close = any(
                math.hypot(cand_x - px, cand_y - py) < spacing
                for px, py in placed_coords
            )
            if too_close:
                continue

            best_x, best_y = cand_x, cand_y
            best_z = sampler.get_height(cand_x, cand_y)
            break

        if best_x is None:
            print(
                f"  [Warning] Rock {idx}: could not place with spacing={spacing:.2f}m "
                f"after {max_attempts} attempts — skipping."
            )
            continue

        rock_entry = {
            "id": idx,
            "name": f"Rock_{idx}",
            "x": float(best_x),
            "y": float(best_y),
            "z": float(best_z),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": float(random.uniform(-math.pi, math.pi)),
            "rock_id": int(random.randint(1, 9)),
            "is_collidable": bool(random.random() < collidable_ratio),
            "is_barrier": False,
            "world_name": str(world_name),
        }
        placed_coords.append((best_x, best_y))
        rock_data_list.append(rock_entry)

    placed = len(rock_data_list)
    print(f"\n  Placed {placed}/{num_rocks} rocks successfully.")

    # ── 5. Save outputs ──────────────────────────────────────────────────────
    obs_array = np.array(rock_data_list, dtype=object)
    np.save(output_file, obs_array)
    print(f"  -> Primary file   : {output_file}")

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamped_file = os.path.join(output_dir, f"obstacle_data_{timestamp}.npy")
    np.save(timestamped_file, obs_array)
    print(f"  -> Timestamped    : {timestamped_file}")

    info_file = os.path.splitext(output_file)[0] + "_info.txt"
    with open(info_file, "w") as f:
        f.write("Obstacle Data Summary\n")
        f.write(f"World Name      : {world_name}\n")
        f.write(f"Heightmap       : {heightmap_path}\n")
        f.write(f"Total Placed    : {placed}\n")
        f.write(f"Density         : {density} rocks/m²\n")
        f.write(f"Collidable Ratio: {collidable_ratio}\n")
        f.write(f"Spacing Min     : {spacing} m\n")
        f.write(f"Min Terrain Z   : {min_terrain_height} m\n")
        f.write(f"Deadends        : {deadends}\n")
        f.write(f"Timestamped File: obstacle_data_{timestamp}.npy\n\n")
        for r in rock_data_list:
            f.write(
                f"[{r['id']:3d}] {r['name']:<12} | Model #{r['rock_id']} | "
                f"Pos: ({r['x']:7.3f}, {r['y']:7.3f}, {r['z']:6.3f}) | "
                f"Collidable: {r['is_collidable']}\n"
            )

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
        help="Ratio of rocks with collision enabled (default: 0.5)",
    )
    parser.add_argument(
        "-s", "--spacing", type=float, default=1.0,
        help="Minimum centre-to-centre spacing between rocks in metres (default: 1.0)",
    )
    parser.add_argument(
        "--min-roughness", type=float, default=0.02,
        help="Min local Z std-dev to accept a cell as rough terrain (default: 0.02). "
             "Increase to exclude flatter areas; decrease to allow gentler slopes.",
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
        help="Override path to heightmap .npz (default: auto-detected from package)",
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Output .npy path (default: obs_data/obstacle_data.npy in package src)",
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
    )


if __name__ == "__main__":
    main()
