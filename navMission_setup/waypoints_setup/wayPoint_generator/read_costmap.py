import sys
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

def inspect_and_plot_costmap(filepath):
    path = Path(filepath)
    if not path.exists():
        print(f"[ERROR] File not found: {path.resolve()}")
        return

    # Load the .npz archive
    data = np.load(path)
    print(f"\n--- Costmap Info: {path.name} ---")
    print(f"Keys found in file: {list(data.keys())}")

    # Auto-detect costmap key ('total', 'costmap', etc.)
    if 'total' in data:
        key = 'total'
    elif 'costmap' in data:
        key = 'costmap'
    else:
        key = data.files[0]

    costmap = data[key]

    print(f"\nUsing array key: '{key}'")
    print(f"Grid Dimensions: {costmap.shape[0]} rows x {costmap.shape[1]} cols (Height x Width)")
    print(f"Data Type:       {costmap.dtype}")
    print(f"Min Cost:        {costmap.min()}")
    print(f"Max Cost:        {costmap.max()}")
    print(f"Average Cost:    {costmap.mean():.4f}")

    # Print spatial & terrain metadata if present
    if 'resolution' in data:
        print(f"Resolution:      {data['resolution']} meters/cell")
    if 'origin_x' in data and 'origin_y' in data:
        print(f"World Origin:    ({data['origin_x']}, {data['origin_y']})")
    if 'heightmap_path' in data:
        print(f"Heightmap Path:  {data['heightmap_path']}")

    # Setup plot visualization
    fig, ax = plt.subplots(figsize=(9, 6))

    # Mask out unknown space (-1) so it renders as grey instead of black
    cmap = plt.cm.inferno.copy()
    cmap.set_bad(color='dimgray')
    masked_costmap = np.ma.masked_equal(costmap, -1)

    img = ax.imshow(masked_costmap, cmap=cmap, origin='lower')
    cbar = plt.colorbar(img, ax=ax, label='Cost Value')

    if (costmap == -1).any():
        cbar.ax.set_xlabel('Grey = Unmapped (-1)', labelpad=10, fontsize=8)

    ax.set_title(f"Costmap: {path.name} | Dimensions: {costmap.shape[0]}x{costmap.shape[1]}")
    ax.set_xlabel("X (Grid Cells)")
    ax.set_ylabel("Y (Grid Cells)")
    ax.grid(True, which='both', color='gray', linestyle=':', linewidth=0.5)

    print("\n[INFO] Close the plot window to exit.")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        target = sys.argv[1]
    else:
        target = input("Enter costmap index (e.g. 1, 2) or full file path: ").strip()

    # Expand single numeric entries (e.g. entering "3" -> "inputs/costmap_3.npz")
    if target.isdigit():
        target = f"inputs/costmap_{target}.npz"

    inspect_and_plot_costmap(target)