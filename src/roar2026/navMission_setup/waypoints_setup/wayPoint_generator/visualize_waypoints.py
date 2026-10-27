"""
visualize_waypoints.py

Step through generated waypoint missions one at a time (5-point sets), each
drawn over the heightmap with obstacles marked.

Usage (run from inside your wayPoint_generator/ package, next to wp_generator.py):

    Interactive, one mission at a time (arrow keys / buttons to navigate):
        python visualize_waypoints.py

    Jump straight to a specific mission:
        python visualize_waypoints.py --index 12

    Skip interactivity entirely and dump every mission as its own PNG:
        python visualize_waypoints.py --save-all outputs/mission_previews

Requires wp_generator.py to be importable (same directory) since it reuses
RoverConfig, Constraints, and _compute_valid_map_mask so the mask you see here
matches exactly what the generator computed.
"""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np

from wp_generator import RoverConfig, Constraints, _compute_valid_map_mask


def load_heightmap(inputs_dir: Path) -> np.ndarray:
    with np.load(inputs_dir / "heightmap.npz") as data:
        for key in data.files:
            arr = data[key]
            if isinstance(arr, np.ndarray) and arr.ndim == 2:
                return arr.astype(np.float32)
    raise ValueError("No 2D heightmap array found in heightmap.npz")


def load_obstacles_as_points(inputs_dir: Path, heightmap: np.ndarray, res: float):
    """Every obstacle projected into GRID coordinates, tagged with whether it's
    collidable/barrier (just for coloring -- both kinds are shown, since
    waypoints are allowed to land on non-collidable ones)."""
    h, w = heightmap.shape
    cx, cy = w / 2.0, h / 2.0
    pts = []
    for obs_path in sorted(inputs_dir.glob("obstacle_data*.npy")):
        raw = np.load(obs_path, allow_pickle=True)
        if isinstance(raw, np.ndarray) and raw.dtype == object and raw.ndim == 0:
            raw = raw.item()
        items = raw.flat if isinstance(raw, np.ndarray) else raw
        for obj in items:
            if not isinstance(obj, dict):
                continue
            px = cx + obj.get("x", 0.0) / res
            py = cy + obj.get("y", 0.0) / res
            blocking = bool(obj.get("is_collidable", True) or obj.get("is_barrier", False))
            pts.append((px, py, blocking))
    return pts


def load_missions(outputs_dir: Path):
    missions = []
    for f in sorted(outputs_dir.glob("wp*.npy")):
        missions.append((f.stem, np.load(f)))
    return missions


def draw_background(ax, heightmap, valid_mask, obstacles):
    hm = np.ma.masked_invalid(np.where(heightmap == -1, np.nan, heightmap))
    im = ax.imshow(hm, origin="lower", cmap="terrain")

    if valid_mask is not None:
        ax.contour(valid_mask.astype(int), levels=[0.5], colors="cyan", linewidths=1.2)
        ax.plot([], [], color="cyan", label="valid mask boundary")

    blocking = [(x, y) for x, y, b in obstacles if b]
    non_blocking = [(x, y) for x, y, b in obstacles if not b]
    if non_blocking:
        nx, ny = zip(*non_blocking)
        ax.scatter(nx, ny, marker="x", c="gray", s=30, label=f"non-collidable ({len(non_blocking)})")
    if blocking:
        bx, by = zip(*blocking)
        ax.scatter(bx, by, marker="x", c="red", s=40, label=f"collidable/barrier ({len(blocking)})")
    return im


def plot_mission(ax, name, wp, h, w):
    ax.plot(wp[:, 0], wp[:, 1], "-o", color="lime", markersize=7, linewidth=2, zorder=5)
    for i, pt in enumerate(wp):
        ax.annotate(str(i), (pt[0], pt[1]), fontsize=9, color="white", weight="bold",
                    xytext=(4, 4), textcoords="offset points", zorder=6)
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_xlabel("grid x")
    ax.set_ylabel("grid y")
    ax.set_title(f"Mission: {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--inputs", default="inputs")
    ap.add_argument("--outputs", default="outputs")
    ap.add_argument("--index", type=int, default=0, help="Mission index to start on")
    ap.add_argument("--save-all", default=None,
                     help="If set, skip the interactive viewer and save every mission as its own PNG in this directory")
    args = ap.parse_args()

    inputs_dir = Path(args.inputs)
    outputs_dir = Path(args.outputs)

    heightmap = load_heightmap(inputs_dir)
    h, w = heightmap.shape

    with open(inputs_dir / "rover_config.json") as f:
        rover_cfg = RoverConfig(**json.load(f))
    with open(inputs_dir / "wp_constraints.json") as f:
        constraints = Constraints(**json.load(f))

    boundary_margin_cells = max(1, int(min(h, w) * (constraints.boundary_margin_ratio or 0.05)))
    valid_mask = _compute_valid_map_mask([heightmap], boundary_margin_cells)

    res = getattr(rover_cfg, "grid_resolution_m", 0.1)
    obstacles = load_obstacles_as_points(inputs_dir, heightmap, res)
    missions = load_missions(outputs_dir)

    if not missions:
        print("No wp*.npy files found in outputs/.")
        return

    if args.save_all:
        save_dir = Path(args.save_all)
        save_dir.mkdir(parents=True, exist_ok=True)
        for name, wp in missions:
            fig, ax = plt.subplots(figsize=(9, 7))
            im = draw_background(ax, heightmap, valid_mask, obstacles)
            fig.colorbar(im, ax=ax, label="elevation (m)", shrink=0.7)
            plot_mission(ax, name, wp, h, w)
            ax.legend(loc="upper right", fontsize=7)
            fig.savefig(save_dir / f"{name}.png", dpi=150, bbox_inches="tight")
            plt.close(fig)
        print(f"Saved {len(missions)} mission previews to {save_dir.resolve()}")
        return

    state = {"i": max(0, min(args.index, len(missions) - 1))}

    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.12)

    def render():
        ax.clear()
        im = draw_background(ax, heightmap, valid_mask, obstacles)
        name, wp = missions[state["i"]]
        plot_mission(ax, f"{name}  ({state['i']+1}/{len(missions)})", wp, h, w)
        ax.legend(loc="upper right", fontsize=7)
        fig.canvas.draw_idle()

    def next_mission(event=None):
        state["i"] = (state["i"] + 1) % len(missions)
        render()

    def prev_mission(event=None):
        state["i"] = (state["i"] - 1) % len(missions)
        render()

    def on_key(event):
        if event.key in ("right", "n", " "):
            next_mission()
        elif event.key in ("left", "p"):
            prev_mission()

    fig.canvas.mpl_connect("key_press_event", on_key)

    ax_prev = fig.add_axes([0.35, 0.02, 0.1, 0.05])
    ax_next = fig.add_axes([0.55, 0.02, 0.1, 0.05])
    btn_prev = Button(ax_prev, "< Prev")
    btn_next = Button(ax_next, "Next >")
    btn_prev.on_clicked(prev_mission)
    btn_next.on_clicked(next_mission)

    render()
    plt.show()


if __name__ == "__main__":
    main()