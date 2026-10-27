import pathlib
#!/usr/bin/env python3
"""
Quick before/after path comparison visualizer.
Overlays original and edited paths on the costmap using matplotlib.
Not part of the main editor — run once, close when done.
"""
import csv, math, os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
COSTMAP_FILE   = str(pathlib.Path(SCRIPT_DIR).parent.parent / 'data' / 'costmap.csv')
ORIGINAL_FILE  = str(pathlib.Path(SCRIPT_DIR).parent.parent / 'data' / 'combined_offline_path.csv')
EDITED_FILE    = str(pathlib.Path(SCRIPT_DIR).parent.parent / 'data' / 'edited_offline_path.csv')
WAYPOINTS_FILE = str(pathlib.Path(SCRIPT_DIR).parent.parent / 'reference' / 'waypoints.csv')

# --------------------------------------------------------------------------
# Load costmap (skip # comment header lines)
# --------------------------------------------------------------------------
def load_costmap(path):
    lines = [l for l in open(path) if not l.startswith('#')]
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            if 'resolution=' in line:
                meta['resolution'] = float(line.split('resolution=')[1].split()[0])
            if 'origin_x=' in line:
                for tok in line.split():
                    if tok.startswith('origin_x='):
                        meta['origin_x'] = float(tok.split('=')[1])
                    if tok.startswith('origin_y='):
                        meta['origin_y'] = float(tok.split('=')[1])

    reader = csv.reader(lines)
    header = next(reader)
    width = len(header) - 1
    rows = []
    for row in reader:
        if not row:
            continue
        vals = row[1:width+1]
        rows.append([int(v) for v in vals])

    resolution = meta.get('resolution', 0.1)
    origin_x   = meta.get('origin_x',   0.0)
    origin_y   = meta.get('origin_y',   0.0)
    return np.array(rows, dtype=np.int16), width, len(rows), resolution, origin_x, origin_y

# --------------------------------------------------------------------------
# Load a path CSV into (x, y) arrays
# --------------------------------------------------------------------------
def load_path(path):
    xs, ys = [], []
    if not os.path.exists(path):
        return xs, ys
    with open(path) as f:
        reader = csv.reader(f)
        next(reader, None)          # skip header
        for row in reader:
            if len(row) >= 2:
                try:
                    xs.append(float(row[0]))
                    ys.append(float(row[1]))
                except ValueError:
                    pass
    return xs, ys

# --------------------------------------------------------------------------
# Load waypoints (x, y, label)
# --------------------------------------------------------------------------
def load_waypoints(path):
    wps = []
    if not os.path.exists(path):
        return wps
    with open(path) as f:
        reader = csv.reader(f)
        for idx, row in enumerate(reader):
            row = [c.strip() for c in row]
            if len(row) >= 2:
                lbl = row[2].strip() if len(row) >= 3 and row[2].strip() else f'WP{idx}'
                wps.append((float(row[0]), float(row[1]), lbl))
    return wps

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
grid, W, H, res, ox, oy = load_costmap(COSTMAP_FILE)

# Build a displayable costmap image (grey = free, black = obstacle, dark-grey = costly)
display = np.zeros((H, W, 3), dtype=np.uint8)
for r in range(H):
    for c in range(W):
        v = grid[r, c]
        if v == -1 or v == 255:
            display[r, c] = (80, 80, 80)       # unknown — dark grey
        elif v >= 90:
            display[r, c] = (20, 20, 20)        # obstacle — near-black
        else:
            intensity = 240 - int((v / 100.0) * 140)
            display[r, c] = (intensity, intensity, intensity)

# World extent for axis labels (matching path_editor top-to-bottom Y axis)
extent = [ox, ox + W * res, oy + H * res, oy]

# Load paths
ox_path, oy_path = load_path(ORIGINAL_FILE)
ex_path, ey_path = load_path(EDITED_FILE)
waypoints = load_waypoints(WAYPOINTS_FILE)

# Path length helper
def path_len(xs, ys):
    return sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(len(xs)-1))

fig, ax = plt.subplots(figsize=(14, 9))
ax.imshow(display, extent=extent, origin='upper', aspect='equal')

# Plot original path
if ox_path:
    ax.plot(ox_path, oy_path, color='#00BFFF', linewidth=1.5,
            label=f'Original  ({path_len(ox_path, oy_path):.1f} m, {len(ox_path)} pts)',
            zorder=3)

# Plot edited path
if ex_path:
    ax.plot(ex_path, ey_path, color='#FF6600', linewidth=2.0, linestyle='--',
            label=f'Edited    ({path_len(ex_path, ey_path):.1f} m, {len(ex_path)} pts)',
            zorder=4)
else:
    print(f"NOTE: {EDITED_FILE} not found — save a path in the editor (press S) first.")

# Overlay waypoint markers
for wx, wy, lbl in waypoints:
    ax.plot(wx, wy, 'r^', markersize=9, zorder=5)
    ax.annotate(lbl, (wx, wy), textcoords='offset points', xytext=(6, 4),
                fontsize=9, fontweight='bold', color='white',
                bbox=dict(boxstyle='round,pad=0.2', fc='black', alpha=0.7))

ax.set_xlabel('World X (m)')
ax.set_ylabel('World Y (m)')
ax.set_title('Path Comparison: Original (blue) vs Edited (orange dashed)')
ax.legend(loc='upper right', framealpha=0.85)
ax.grid(True, linestyle=':', linewidth=0.4, alpha=0.6)
plt.tight_layout()
plt.show()
