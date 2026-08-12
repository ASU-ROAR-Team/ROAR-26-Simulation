#!/usr/bin/env python3
"""
Heightmap diagnostic — run this BEFORE touching generator.py again.

Usage:
    python3 inspect_heightmap.py /path/to/marsyard_heightmap.npz

Prints the Z-value distribution of the heightmap and, for a range of
candidate min_terrain_height thresholds, what percentage of cells would
pass. Use this to pick a threshold that actually matches your visible
"valid mask" area instead of guessing.
"""
import sys
import numpy as np


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 inspect_heightmap.py /path/to/heightmap.npz")
        sys.exit(1)

    path = sys.argv[1]
    with np.load(path) as data:
        grid = data["grid"].astype(np.float64)

    valid_mask = ~np.isnan(grid)
    valid_z = grid[valid_mask]

    print("=" * 60)
    print(f"Heightmap: {path}")
    print("=" * 60)
    print(f"Grid shape        : {grid.shape}")
    print(f"Total cells       : {grid.size}")
    print(f"Non-NaN cells     : {valid_mask.sum()} "
          f"({100 * valid_mask.sum() / grid.size:.1f}%)")
    print()
    print(f"Z min             : {valid_z.min():.4f}")
    print(f"Z max             : {valid_z.max():.4f}")
    print(f"Z mean            : {valid_z.mean():.4f}")
    print(f"Z std             : {valid_z.std():.4f}")
    print()
    print("Z percentiles:")
    for p in [1, 5, 10, 25, 50, 75, 90, 95, 99]:
        print(f"  {p:>3}th percentile : {np.percentile(valid_z, p):.4f}")
    print()

    print("=" * 60)
    print("How much area passes at different min_terrain_height values:")
    print("=" * 60)
    # Build candidate thresholds from the actual data range, not guesses
    candidates = sorted(set(
        [0.0, 0.15] +
        [float(np.percentile(valid_z, p)) for p in [1, 5, 10, 25, 50, 75, 90]]
    ))
    for thresh in candidates:
        pass_count = np.sum(valid_mask & (grid >= thresh))
        pct = 100 * pass_count / grid.size
        print(f"  min_terrain_height >= {thresh:8.4f}  ->  {pass_count:>8} cells "
              f"({pct:5.1f}% of total grid)")

    print()
    print("Pick a threshold above whose %% roughly matches the cyan 'valid mask' "
          "area you see in your mission-planning plot. If min_terrain_height=0.15 "
          "shows a low percentage compared to the others, that confirms it's the "
          "bottleneck.")


if __name__ == "__main__":
    main()
