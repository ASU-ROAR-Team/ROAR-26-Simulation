import pathlib
#!/usr/bin/env python3
import argparse
import csv
import math
import numpy as np
from scipy.ndimage import gaussian_filter

def main():
    ap = argparse.ArgumentParser(
        description="Convert a heightmap (.npz or .csv) to a D* / Nav2-compatible costmap CSV."
    )
    ap.add_argument("-i", "--input", default=str(pathlib.Path(__file__).resolve().parent.parent.parent / "data" / "heightmap_world.npz"), help="Path to input heightmap file (.npz or grid .csv)")
    ap.add_argument("-o", "--output", default=str(pathlib.Path(__file__).resolve().parent.parent.parent / "data" / "costmap.csv"), help="Path to output costmap CSV (default: ../../costmap.csv)")
    ap.add_argument("--max-slope", type=float, default=15.0, help="Slope angle (degrees) above which a cell is lethal (default: 15.0)")
    ap.add_argument("--max-height-diff", type=float, default=0.30, help="Maximum height difference (meters) between adjacent cells before it is lethal (default: 0.30)")
    ap.add_argument("--resolution", type=float, default=None, help="Override resolution in meters/cell (inferred from file if not provided)")
    ap.add_argument("--smooth", type=float, default=1.0, help="Sigma for Gaussian smoothing of the heightmap (default: 1.0, 0.0 to disable)")
    args = ap.parse_args()

    # ---- Load heightmap ----------------------------------------------------
    print(f"Loading heightmap from {args.input}...")
    if args.input.endswith(".npz"):
        with np.load(args.input) as data:
            xs = data["xs"]
            ys = data["ys"]
            grid = data["grid"]
            res = float(data["resolution"]) if "resolution" in data else None
    else:
        # Load from CSV format
        # Try reading grid CSV
        try:
            res = 1.0
            origin_x = 0.0
            origin_y = 0.0
            with open(args.input) as f:
                for line in f:
                    if not line.startswith("#"):
                        break
                    if "resolution=" in line:
                        res = float(line.split("resolution=")[1].split()[0])
                    if "origin_x=" in line:
                        parts = line.split()
                        for p in parts:
                            if p.startswith("origin_x="):
                                origin_x = float(p.split("=")[1])
                            elif p.startswith("origin_y="):
                                origin_y = float(p.split("=")[1])
            grid = np.genfromtxt(args.input, delimiter=",", comments="#")
            if grid.ndim == 1:
                grid = grid.reshape(1, -1)
            ny, nx = grid.shape
            xs = origin_x + np.arange(nx) * res
            ys = origin_y + np.arange(ny) * res
        except Exception as e:
            print(f"Error loading CSV heightmap: {e}")
            return

    if args.resolution is not None:
        res = args.resolution
    elif res is None:
        # Infer resolution
        if len(xs) > 1:
            res = float(xs[1] - xs[0])
        else:
            res = 0.1
            print(f"Warning: Resolution could not be inferred, using default: {res} m/cell")

    # Apply Gaussian smoothing to filter out high-frequency mesh triangulation noise
    if args.smooth > 0.0:
        print(f"Applying Gaussian smoothing with sigma={args.smooth}...")
        grid = gaussian_filter(grid, sigma=args.smooth)

    ny, nx = grid.shape
    print(f"Loaded heightmap grid of size {nx}x{ny} (Resolution: {res:.4f} m/cell)")

    # ---- Calculate Slopes & Obstacles ---------------------------------------
    print("Computing slopes and traversability costs...")
    costmap = np.zeros((ny, nx), dtype=np.int32)

    # Pre-calculate neighbor offsets
    # 8-connectivity
    neighbors = [
        (-1, 0, 1.0),   # Up
        (1, 0, 1.0),    # Down
        (0, -1, 1.0),   # Left
        (0, 1, 1.0),    # Right
        (-1, -1, math.sqrt(2.0)), # Diagonals
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0))
    ]

    for y in range(ny):
        for x in range(nx):
            z_curr = grid[y, x]
            if np.isnan(z_curr):
                costmap[y, x] = -1 # Unknown
                continue

            max_cell_slope = 0.0
            is_lethal = False

            for dy, dx, dist_mult in neighbors:
                ny_idx = y + dy
                nx_idx = x + dx
                if 0 <= ny_idx < ny and 0 <= nx_idx < nx:
                    z_neigh = grid[ny_idx, nx_idx]
                    if np.isnan(z_neigh):
                        continue
                    
                    diff = abs(z_neigh - z_curr)
                    dist = res * dist_mult

                    # 1. Height step / cliff check (like occupancy_threshold in active_mapping)
                    if diff > args.max_height_diff:
                        is_lethal = True
                        break

                    # 2. Slope check
                    slope = diff / dist
                    angle = math.atan(slope) * 180.0 / math.pi
                    if angle > max_cell_slope:
                        max_cell_slope = angle

            if is_lethal or max_cell_slope >= args.max_slope:
                costmap[y, x] = 100 # Obstacle
            else:
                # Map slope 0 -> max_slope to cost 0 -> 99
                costmap[y, x] = int((max_cell_slope / args.max_slope) * 99)

    # ---- Save Costmap CSV --------------------------------------------------
    print(f"Saving costmap to {args.output}...")
    origin_x = float(xs[0])
    origin_y = float(ys[0])
    with open(args.output, "w") as f:
        f.write(f"# resolution={res}\n")
        f.write(f"# origin_x={origin_x} origin_y={origin_y}\n")
        writer = csv.writer(f)
        
        # Write header row: ,0,1,2,...,nx-1
        header = [""] + [str(i) for i in range(nx)]
        writer.writerow(header)
        
        # Write data rows: row_idx,val0,val1,...
        for y in range(ny):
            row = [str(y)] + [str(costmap[y, x]) for x in range(nx)]
            writer.writerow(row)

    n_lethal = int((costmap == 100).sum())
    n_unknown = int((costmap == -1).sum())
    n_free = int((costmap == 0).sum())
    n_rough = nx * ny - n_lethal - n_unknown - n_free
    print(f"Successfully generated costmap:")
    print(f"  Total cells: {nx * ny}")
    print(f"  Lethal obstacles (100): {n_lethal}")
    print(f"  Unknown cells (-1): {n_unknown}")
    print(f"  Rough terrain (1..99): {n_rough}")
    print(f"  Flat/Free cells (0): {n_free}")

if __name__ == "__main__":
    main()
