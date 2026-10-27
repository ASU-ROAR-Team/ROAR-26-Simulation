import os
import sys
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description="Preview Mars Yard World and Waypoint Mission")
    parser.add_argument("--world", type=int, required=True, help="World number (1, 2, or 3)")
    parser.add_argument("--wp", type=str, required=True, help="Waypoint number (e.g. 0, 00, 1, 01)")
    parser.add_argument("--save", type=str, help="Path to save the PNG instead of showing it", default=None)
    args = parser.parse_args()
    
    script_dir = Path(__file__).resolve().parent
    ws_root = Path(os.environ.get("ROAR_WS", script_dir.parents[2])).resolve()
    sim_dir = ws_root / "src" / "roar2026"
    mission_dir = ws_root / "src" / "navigation"
    
    # Paths
    hm_path = sim_dir / "dev_environment" / "worlds" / f"world{args.world}" / "heightmap.npz"
    obs_path = sim_dir / "dev_environment" / "worlds" / f"world{args.world}" / "obstacle_data.npy"
    
    wp_int = int(args.wp)
    wp_candidates = [
        mission_dir / "missions" / f"mission_{args.world}" / "waypoints" / f"wp{wp_int:02d}" / f"wp{wp_int:02d}.npy",
        mission_dir / "missions" / f"mission_{args.world}" / "waypoints" / f"wp{args.wp}" / f"wp{args.wp}.npy",
        mission_dir / "missions" / f"mission_{args.world}" / "waypoints" / f"wp{wp_int}" / f"wp{wp_int}.npy",
    ]
    wp_path = next((p for p in wp_candidates if p.exists()), None)
    
    if not wp_path:
        print(f"Error: Mission file not found for world {args.world} wp {args.wp}")
        return
        
    if not hm_path.exists() or not obs_path.exists():
        print(f"Error: World data not found for world {args.world} at {hm_path}")
        return

    # Load Heightmap and Origins
    hm = None
    origin_x, origin_y = 0.0, 0.0
    with np.load(hm_path) as data:
        if 'origin_x' in data: origin_x = float(data['origin_x'])
        if 'origin_y' in data: origin_y = float(data['origin_y'])
        for k in data.files:
            if k == 'grid' or data[k].ndim == 2:
                hm = data[k].astype(np.float32)
                break
                
    if hm is None:
        print("Error reading heightmap grid")
        return

    h, w = hm.shape

    # Load obstacles
    obs = np.load(obs_path, allow_pickle=True)
    if isinstance(obs, np.ndarray) and obs.ndim == 0:
        obs = obs.item()
    
    # Load waypoints
    wp = np.load(wp_path)
    
    # Draw plot
    fig, ax = plt.subplots(figsize=(10, 8))
    fig.canvas.manager.set_window_title(f"Mission Preview - World {args.world} WP {args.wp}")
    
    # Background
    masked_hm = np.ma.masked_invalid(np.where(hm == -1, np.nan, hm))
    im = ax.imshow(masked_hm, origin="lower", cmap="terrain")
    fig.colorbar(im, ax=ax, label="Elevation (m)", shrink=0.7)
    
    # Obstacles
    res = 0.25 # standard grid resolution
    bx, by = [], []
    nx, ny = [], []
    
    items = obs.flat if isinstance(obs, np.ndarray) else obs
    for obj in items:
        if not isinstance(obj, dict): continue
        px = (obj.get("x", 0.0) - origin_x) / res
        py = (obj.get("y", 0.0) - origin_y) / res
        if obj.get("is_collidable", True) or obj.get("is_barrier", False):
            bx.append(px)
            by.append(py)
        else:
            nx.append(px)
            ny.append(py)
            
    if nx:
        ax.scatter(nx, ny, marker="x", c="magenta", s=20, label=f"Safe Pebbles ({len(nx)})", alpha=0.8)
    if bx:
        ax.scatter(bx, by, marker="X", c="red", s=35, label=f"Collidable Obstacles ({len(bx)})", alpha=1.0)
        
    # Path
    plot_y = h - 1 - wp[:, 1]
    ax.plot(wp[:, 0], plot_y, "-o", color="lime", markersize=8, linewidth=2, zorder=5, label="Driving Path")
    for i, pt in enumerate(wp):
        py = h - 1 - pt[1]
        label = f"Point {i+1}"
        color = "#00FF00" if i == 0 else ("#FF0000" if i == len(wp) - 1 else "white")
        ax.annotate(f"  {label}  ", (pt[0], py), fontsize=11, color=color, weight="bold", zorder=6,
                    bbox=dict(facecolor='black', edgecolor='none', alpha=0.7, pad=3))

    # Rover Start Position
    spawn_px = (0.0 - origin_x) / res
    spawn_py = (0.0 - origin_y) / res
    ax.plot(spawn_px, spawn_py, marker="*", color="cyan", markersize=18, markeredgecolor="black", zorder=10, label="Rover Start (0,0)")
    ax.annotate("  ROVER START  ", (spawn_px, spawn_py), fontsize=11, color="black", weight="bold", zorder=11,
                bbox=dict(facecolor='cyan', edgecolor='black', alpha=0.9, pad=3))
        
    ax.set_xlim(0, w)
    ax.set_ylim(0, h)
    ax.set_xlabel("Grid X")
    ax.set_ylabel("Grid Y")
    ax.set_title(f"World {args.world} — Mission wp{args.wp}", fontsize=14, weight="bold", pad=20)
    ax.legend(loc="upper right")
    
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"Saved preview image to: {args.save}")
    else:
        plt.show()

if __name__ == "__main__":
    main()
