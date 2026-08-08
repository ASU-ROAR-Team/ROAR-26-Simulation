#!/usr/bin/env python3
"""Visualize a height map file produced by generate_heightmap.py.

Supports every format that script can write:
  - .npz: arrays 'xs', 'ys', 'grid' (plus 'resolution', 'origin_x', 'origin_y').
  - grid CSV (default CSV format): a matrix of z-values, one row per Y, one
    column per X, with a '#'-commented metadata header (resolution, origin_x/y).
  - long-format CSV (--long-format in generate_heightmap.py): 'x,y,z' rows.

Format is picked automatically from the file (.npz extension, or the first
CSV data line).

Usage:
    ./visualize_heightmap.py heightmap.npz
    ./visualize_heightmap.py heightmap.csv --type 3d
    ./visualize_heightmap.py heightmap.csv --type contour -o contour.png
    ./visualize_heightmap.py heightmap.npz --hillshade --cmap terrain
"""
import argparse
import re
import sys

import numpy as np


def read_grid_csv(path):
    resolution = 1.0
    origin_x = 0.0
    origin_y = 0.0
    with open(path) as f:
        for line in f:
            if not line.startswith("#"):
                break
            m = re.search(r"resolution=([\d.eE+-]+)", line)
            if m:
                resolution = float(m.group(1))
            m = re.search(r"origin_x=([\d.eE+-]+)\s+origin_y=([\d.eE+-]+)", line)
            if m:
                origin_x, origin_y = float(m.group(1)), float(m.group(2))

    grid = np.genfromtxt(path, delimiter=",", comments="#")
    if grid.ndim == 1:
        grid = grid.reshape(1, -1)
    ny, nx = grid.shape
    xs = origin_x + np.arange(nx) * resolution
    ys = origin_y + np.arange(ny) * resolution
    return xs, ys, grid


def read_long_csv(path):
    data = np.genfromtxt(path, delimiter=",", names=True)
    x, y, z = data["x"], data["y"], data["z"]
    xs = np.sort(np.unique(x))
    ys = np.sort(np.unique(y))
    ix = np.searchsorted(xs, x)
    iy = np.searchsorted(ys, y)
    grid = np.full((len(ys), len(xs)), np.nan)
    grid[iy, ix] = z
    return xs, ys, grid


def read_npz(path):
    with np.load(path) as data:
        return data["xs"], data["ys"], data["grid"]


def load_heightmap(path):
    if path.lower().endswith(".npz"):
        return read_npz(path)

    with open(path) as f:
        for line in f:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            first_line = stripped
            break
        else:
            sys.exit(f"'{path}' has no data rows")

    if first_line.lower().replace(" ", "") == "x,y,z":
        return read_long_csv(path)
    return read_grid_csv(path)


def plot_heatmap(ax, xs, ys, grid, cmap, hillshade):
    masked = np.ma.masked_invalid(grid)
    extent = [xs[0], xs[-1], ys[0], ys[-1]]
    if hillshade:
        from matplotlib.colors import LightSource
        ls = LightSource(azdeg=315, altdeg=45)
        filled = np.ma.filled(masked, np.nanmin(grid) if np.isfinite(grid).any() else 0.0)
        rgb = ls.shade(filled, cmap=plt_get_cmap(cmap), vert_exag=1.0, blend_mode="soft")
        im = ax.imshow(rgb, origin="lower", extent=extent)
        # add an invisible mappable for the colorbar to reference real z values
        sm = plt_scalarmappable(cmap, np.nanmin(grid), np.nanmax(grid))
        return im, sm
    else:
        im = ax.imshow(masked, origin="lower", extent=extent, cmap=cmap)
        return im, im


def plt_get_cmap(name):
    import matplotlib.pyplot as plt
    return plt.get_cmap(name)


def plt_scalarmappable(cmap, vmin, vmax):
    import matplotlib.pyplot as plt
    import matplotlib.cm as cm
    norm = plt.Normalize(vmin=vmin, vmax=vmax)
    return cm.ScalarMappable(norm=norm, cmap=cmap)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="height map CSV file (grid or long format)")
    ap.add_argument("-o", "--output", help="save figure to this path instead of showing it interactively")
    ap.add_argument("--type", choices=["heatmap", "contour", "3d"], default="heatmap")
    ap.add_argument("--cmap", default="terrain", help="matplotlib colormap name (default: terrain)")
    ap.add_argument("--hillshade", action="store_true", help="shade the heatmap using a synthetic sun angle (heatmap type only)")
    ap.add_argument("--levels", type=int, default=20, help="number of contour levels (contour type only)")
    ap.add_argument("--title", default=None)
    ap.add_argument("--dpi", type=int, default=150)
    args = ap.parse_args()

    import matplotlib
    if args.output:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs, ys, grid = load_heightmap(args.input)
    title = args.title or args.input

    if args.type == "3d":
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111, projection="3d")
        gx, gy = np.meshgrid(xs, ys)
        masked = np.ma.masked_invalid(grid)
        surf = ax.plot_surface(gx, gy, masked, cmap=args.cmap, linewidth=0, antialiased=True)
        ax.set_zlabel("Z (m)")
        fig.colorbar(surf, shrink=0.6, label="Elevation (m)")
    else:
        fig, ax = plt.subplots(figsize=(8, 7))
        if args.type == "contour":
            masked = np.ma.masked_invalid(grid)
            cs = ax.contourf(xs, ys, masked, levels=args.levels, cmap=args.cmap)
            ax.contour(xs, ys, masked, levels=args.levels, colors="k", linewidths=0.3, alpha=0.5)
            fig.colorbar(cs, ax=ax, label="Elevation (m)")
            ax.set_aspect("equal")
        else:
            im, mappable = plot_heatmap(ax, xs, ys, grid, args.cmap, args.hillshade)
            fig.colorbar(mappable, ax=ax, label="Elevation (m)")
            ax.set_aspect("equal")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(title)
    fig.tight_layout()

    if args.output:
        fig.savefig(args.output, dpi=args.dpi)
        print(f"Saved {args.type} plot to {args.output}", file=sys.stderr)
    else:
        plt.show()


if __name__ == "__main__":
    main()
