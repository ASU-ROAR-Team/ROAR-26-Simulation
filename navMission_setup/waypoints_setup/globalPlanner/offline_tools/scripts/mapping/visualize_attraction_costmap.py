#!/usr/bin/env python3
"""Path Attraction Costmap Visualizer.

Visualizes the pure negative-cost path attraction layer overlaid on top of the terrain costmap
and offline path trajectory.
"""

import os
import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt

def load_csv_costmap(csv_path):
    if not os.path.exists(csv_path):
        return None, {}
    meta = {'resolution': 0.1, 'origin_x': -4.552489, 'origin_y': -10.339172}
    lines = []
    with open(csv_path, 'r') as f:
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
    if not lines:
        return None, meta

    reader = csv.reader(lines)
    header = next(reader, None)
    width = len([h for h in header[1:] if h.strip() != ''])
    grid_rows = []
    for row in reader:
        vals = [float(v.strip()) if v.strip() else 0.0 for v in row[1:width + 1]]
        if len(vals) < width: vals.extend([0.0] * (width - len(vals)))
        grid_rows.append(vals)

    grid = np.array(grid_rows, dtype=np.float32)
    meta['width'] = width
    meta['height'] = len(grid_rows)
    return grid, meta

def load_path(path_csv):
    if not os.path.exists(path_csv):
        return [], []
    xs, ys = [], []
    with open(path_csv, 'r') as f:
        reader = csv.reader(f)
        next(reader, None)
        for row in reader:
            if len(row) >= 2:
                try:
                    xs.append(float(row[0]))
                    ys.append(float(row[1]))
                except ValueError:
                    pass
    return xs, ys

def main():
    _here = os.path.dirname(os.path.abspath(__file__))
    cwd = os.getcwd()

    ap = argparse.ArgumentParser(description="Visualize Path Attraction Costmap Layer.")
    ap.add_argument("-a", "--attraction", default=None, help="Path to attraction costmap CSV")
    ap.add_argument("-t", "--terrain", default=None, help="Path to terrain costmap CSV")
    ap.add_argument("-p", "--path", default=None, help="Path to trajectory CSV")
    ap.add_argument("-o", "--output", default=None, help="Output PNG visualization path")
    args = ap.parse_args()

    attr_csv = args.attraction or os.path.join(cwd, 'data', 'combined_offline_path_attraction_costmap.csv')
    terrain_csv = args.terrain or os.path.join(cwd, 'data', 'costmap.csv')
    path_csv = args.path or os.path.join(cwd, 'data', 'combined_offline_path.csv')

    if not os.path.exists(attr_csv):
        attr_csv = os.path.join(_here, '..', '..', 'data', 'combined_offline_path_attraction_costmap.csv')
    if not os.path.exists(terrain_csv):
        terrain_csv = os.path.join(_here, '..', '..', 'data', 'costmap.csv')
    if not os.path.exists(path_csv):
        path_csv = os.path.join(_here, '..', '..', 'data', 'combined_offline_path.csv')

    attr_grid, attr_meta = load_csv_costmap(attr_csv)
    terrain_grid, terrain_meta = load_csv_costmap(terrain_csv)
    px, py = load_path(path_csv)

    if attr_grid is None:
        print(f"Error: Could not load attraction costmap from {attr_csv}")
        return

    res = attr_meta.get('resolution', 0.1)
    ox = attr_meta.get('origin_x', -4.55)
    oy = attr_meta.get('origin_y', -10.34)
    H, W = attr_grid.shape
    extent = [ox, ox + W * res, oy + H * res, oy]

    fig, axes = plt.subplots(1, 2, figsize=(16, 7), sharex=True, sharey=True)

    # Left Plot: Pure Path Attraction Layer (Heatmap)
    ax1 = axes[0]
    im1 = ax1.imshow(attr_grid, cmap='coolwarm', extent=extent, origin='upper', vmin=-30.0, vmax=0.0)
    ax1.set_title("Path Attraction Costmap Layer (Negative Bonus Cost)", fontsize=12, fontweight='bold')
    ax1.set_xlabel("X Coordinate (m)")
    ax1.set_ylabel("Y Coordinate (m)")
    ax1.grid(True, alpha=0.3, linestyle='--')
    if px and py:
        ax1.plot(px, py, 'w--', linewidth=1.5, label='Offline Path')
        ax1.legend(loc='upper right')
    cbar1 = fig.colorbar(im1, ax=ax1, fraction=0.046, pad=0.04)
    cbar1.set_label("Attraction Bonus Cost (units)", fontsize=10)

    # Right Plot: Combined Overlay (Terrain Costmap + Path Attraction Corridor)
    ax2 = axes[1]
    if terrain_grid is not None:
        ax2.imshow(terrain_grid, cmap='gray_r', extent=extent, origin='upper', alpha=0.6)

    masked_attr = np.ma.masked_where(attr_grid == 0, attr_grid)
    im2 = ax2.imshow(masked_attr, cmap='spring', extent=extent, origin='upper', alpha=0.85, vmin=-30.0, vmax=0.0)
    ax2.set_title("Terrain Costmap + Attraction Corridor Overlay", fontsize=12, fontweight='bold')
    ax2.set_xlabel("X Coordinate (m)")
    ax2.grid(True, alpha=0.3, linestyle='--')
    if px and py:
        ax2.plot(px, py, 'c-', linewidth=2.0, label='Offline Path Centerline')
        ax2.scatter([px[0], px[-1]], [py[0], py[-1]], c='red', s=60, zorder=5, label='Start/End')
        ax2.legend(loc='upper right')
    cbar2 = fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)
    cbar2.set_label("Attraction Bonus (-30 to 0)", fontsize=10)

    plt.suptitle("Marsyard Path Attraction Costmap Layer Verification", fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout()

    out_png = args.output or os.path.join(os.path.dirname(attr_csv), "attraction_costmap_visualization.png")
    plt.savefig(out_png, dpi=200, bbox_inches='tight')
    print(f"Saved visualization image to: {out_png}")
    plt.show()

if __name__ == '__main__':
    main()
