#!/usr/bin/env python3
"""Standalone Path Attraction Costmap Layer Generator.

Calculates a pure negative-cost attraction layer along an offline path CSV:
  - Max negative bonus along path centerline (default: -30)
  - Gaussian decay from centerline to corridor_radius (default: 1.5m)
  - 0 cost for all cells outside corridor
Exports CSV (with comment headers) and NPZ formats.
"""

import os
import csv
import math
import argparse
import numpy as np

def load_path_csv(path_file):
    wps = []
    if not os.path.exists(path_file):
        return wps
    with open(path_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) >= 2:
                try:
                    wps.append((float(row[0]), float(row[1])))
                except ValueError:
                    pass
    return wps

def load_costmap_meta(costmap_file):
    meta = {'resolution': 0.1, 'origin_x': -4.552489, 'origin_y': -10.339172, 'width': 419, 'height': 274}
    if not os.path.exists(costmap_file):
        return meta
    lines = []
    with open(costmap_file, 'r') as f:
        for line in f:
            if line.startswith('#'):
                if 'resolution=' in line:
                    meta['resolution'] = float(line.split('resolution=')[1].split()[0])
                if 'origin_x=' in line:
                    for tok in line.split():
                        if tok.startswith('origin_x='): meta['origin_x'] = float(tok.split('=')[1])
                        if tok.startswith('origin_y='): meta['origin_y'] = float(tok.split('=')[1])
            else:
                lines.append(line)
    if lines:
        reader = csv.reader(lines)
        header = next(reader, None)
        if header:
            meta['width'] = len([h for h in header[1:] if h.strip() != ''])
            meta['height'] = len(lines)
    return meta

def generate_attraction_costmap(path_pts, meta, max_bonus=30.0, corridor_radius=1.5, sigma=0.5):
    res = meta['resolution']
    ox = meta['origin_x']
    oy = meta['origin_y']
    width = meta['width']
    height = meta['height']

    grid = np.zeros((height, width), dtype=np.float32)
    if not path_pts or len(path_pts) == 0:
        return grid

    # Create meshgrid of world coordinates
    x_coords = ox + (np.arange(width) + 0.5) * res
    y_coords = oy + (np.arange(height) + 0.5) * res
    grid_x, grid_y = np.meshgrid(x_coords, y_coords)

    P = np.array(path_pts, dtype=np.float32)
    if len(P) == 1:
        dist_sq = (grid_x - P[0, 0])**2 + (grid_y - P[0, 1])**2
        min_dist = np.sqrt(dist_sq)
    else:
        P1 = P[:-1]
        P2 = P[1:]
        dP = P2 - P1
        len_sq = np.sum(dP**2, axis=1)
        len_sq[len_sq == 0] = 1e-6

        min_dist_sq = np.full((height, width), np.inf, dtype=np.float32)

        for i in range(len(P1)):
            x1, y1 = P1[i]
            dx, dy = dP[i]
            l_sq = len_sq[i]

            t = np.clip(((grid_x - x1) * dx + (grid_y - y1) * dy) / l_sq, 0.0, 1.0)
            proj_x = x1 + t * dx
            proj_y = y1 + t * dy
            d_sq = (grid_x - proj_x)**2 + (grid_y - proj_y)**2
            min_dist_sq = np.minimum(min_dist_sq, d_sq)

        min_dist = np.sqrt(min_dist_sq)

    mask = min_dist <= corridor_radius
    grid[mask] = -max_bonus * np.exp(- (min_dist[mask]**2) / (2.0 * sigma * sigma))
    return np.round(grid, 2)

def export_attraction_costmap(grid, meta, out_csv_path):
    os.makedirs(os.path.dirname(out_csv_path), exist_ok=True)
    height, width = grid.shape
    res = meta['resolution']
    ox = meta['origin_x']
    oy = meta['origin_y']

    with open(out_csv_path, 'w', newline='') as f:
        f.write(f"# resolution={res}\n")
        f.write(f"# origin_x={ox} origin_y={oy}\n")
        writer = csv.writer(f)
        header = [''] + [str(i) for i in range(width)]
        writer.writerow(header)
        for y in range(height):
            row = [str(y)] + [f"{grid[y, x]:.2f}" for x in range(width)]
            writer.writerow(row)

    out_npz_path = os.path.splitext(out_csv_path)[0] + '.npz'
    np.savez_compressed(
        out_npz_path,
        grid=grid,
        resolution=res,
        origin_x=ox,
        origin_y=oy,
        width=width,
        height=height
    )
    print(f"Exported Path Attraction Costmap to:")
    print(f"  - CSV: {out_csv_path}")
    print(f"  - NPZ: {out_npz_path}")

def main():
    _here = os.path.dirname(os.path.abspath(__file__))
    cwd = os.getcwd()

    ap = argparse.ArgumentParser(description="Generate pure negative-cost path attraction layer.")
    ap.add_argument("-i", "--input", default=None, help="Input path CSV file")
    ap.add_argument("-o", "--output", default=None, help="Output attraction costmap CSV file")
    ap.add_argument("-c", "--costmap", default=None, help="Reference terrain costmap CSV for metadata")
    ap.add_argument("--max-bonus", type=float, default=30.0, help="Maximum negative cost bonus at path center (default: 30)")
    ap.add_argument("--corridor-radius", type=float, default=1.5, help="Radius of attraction corridor in meters (default: 1.5)")
    ap.add_argument("--sigma", type=float, default=0.5, help="Gaussian decay sigma in meters (default: 0.5)")
    args = ap.parse_args()

    input_path = args.input or os.path.join(cwd, 'data', 'combined_offline_path.csv')
    costmap_ref = args.costmap or os.path.join(cwd, 'data', 'costmap.csv')

    if not os.path.exists(costmap_ref):
        costmap_ref = os.path.join(_here, '..', '..', 'data', 'costmap.csv')

    meta = load_costmap_meta(costmap_ref)
    path_pts = load_path_csv(input_path)

    if not args.output:
        stem = os.path.splitext(os.path.basename(input_path))[0]
        output_csv = os.path.join(os.path.dirname(input_path), f"{stem}_attraction_costmap.csv")
    else:
        output_csv = args.output

    print(f"Generating attraction costmap layer for path: {input_path}")
    print(f"  - Max Bonus: -{args.max_bonus} cost")
    print(f"  - Corridor Radius: {args.corridor_radius} m")
    print(f"  - Gaussian Sigma: {args.sigma} m")

    grid = generate_attraction_costmap(path_pts, meta, args.max_bonus, args.corridor_radius, args.sigma)
    export_attraction_costmap(grid, meta, output_csv)

if __name__ == '__main__':
    main()
