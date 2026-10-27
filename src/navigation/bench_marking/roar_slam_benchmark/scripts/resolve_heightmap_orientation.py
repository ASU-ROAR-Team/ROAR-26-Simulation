#!/usr/bin/env python3

"""
Heightmap Orientation Resolver (One-off Script)
Tests the 4 candidate transforms (identity, flipud, fliplr, rot180) and 
rotations (0, 90 CW, 90 CCW) against a set of known ground-truth obstacle positions.

Usage:
  python3 resolve_heightmap_orientation.py <path_to_heightmap.npy or .npz> <path_to_ground_truth_obstacles.csv>

Ground truth obstacles CSV should have at least: `x, y` columns in world coordinates.
"""

import sys
import numpy as np
import argparse
import pandas as pd

def transform_grid(grid, transform_type):
    if transform_type == 'identity':
        return grid
    elif transform_type == 'flipud':
        return np.flipud(grid)
    elif transform_type == 'fliplr':
        return np.fliplr(grid)
    elif transform_type == 'rot180':
        return np.rot90(grid, 2)
    elif transform_type == 'rot90_cw':
        return np.rot90(grid, -1)
    elif transform_type == 'rot90_ccw':
        return np.rot90(grid, 1)
    elif transform_type == 'transpose':
        return grid.T
    return grid

def compute_alignment_score(heightmap, res, origin_x, origin_y, obstacles_xy, transform_type):
    grid = transform_grid(heightmap, transform_type)
    
    total_elevation = 0.0
    valid_points = 0
    
    rows, cols = grid.shape
    
    for x, y in obstacles_xy:
        # Convert world (x,y) to grid (row, col)
        # Using standard mapping: x = col * res + origin_x, y = row * res + origin_y
        col = int(round((x - origin_x) / res))
        row = int(round((y - origin_y) / res))
        
        if 0 <= row < rows and 0 <= col < cols:
            # Assuming rocks have higher elevation than surrounding terrain
            elevation = grid[row, col]
            total_elevation += elevation
            valid_points += 1
            
    if valid_points == 0:
        return 0.0
        
    return total_elevation / valid_points

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("heightmap_path", help="Path to the heightmap .npy or .npz file")
    parser.add_argument("obstacles_csv", help="Path to ground truth obstacles CSV containing 'x' and 'y' columns")
    parser.add_argument("--res", type=float, default=0.1, help="Map resolution (m/cell)")
    parser.add_argument("--origin_x", type=float, default=-10.0, help="Map origin X (m)")
    parser.add_argument("--origin_y", type=float, default=-10.0, help="Map origin Y (m)")
    args = parser.parse_args()

    # Load heightmap
    try:
        data = np.load(args.heightmap_path)
        if isinstance(data, np.lib.npyio.NpzFile):
            # Assume the main array is the first one if not specified
            heightmap = data[data.files[0]]
        else:
            heightmap = data
    except Exception as e:
        print(f"Error loading heightmap: {e}")
        return

    # Load obstacles
    try:
        df = pd.read_csv(args.obstacles_csv)
        obstacles_xy = df[['x', 'y']].values
    except Exception as e:
        print(f"Error loading obstacles CSV: {e}")
        return

    transforms = ['identity', 'flipud', 'fliplr', 'rot180', 'rot90_cw', 'rot90_ccw', 'transpose']
    
    print(f"Testing {len(transforms)} candidate transforms...")
    print("-" * 40)
    
    scores = {}
    for t in transforms:
        score = compute_alignment_score(heightmap, args.res, args.origin_x, args.origin_y, obstacles_xy, t)
        scores[t] = score
        print(f"Transform: {t:<15} | Avg Elevation at Obstacles: {score:.4f}")
        
    best = max(scores, key=scores.get)
    print("-" * 40)
    print(f"BEST MATCH: {best}")
    print("Highest average elevation at known obstacle centers indicates the correct alignment.")

if __name__ == "__main__":
    main()
