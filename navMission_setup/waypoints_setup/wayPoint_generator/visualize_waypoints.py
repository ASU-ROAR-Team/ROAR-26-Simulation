import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import re
import signal
import json
from pathlib import Path

signal.signal(signal.SIGINT, signal.SIG_DFL)


def plot_interactive_waypoints():
    # Let Ctrl+C in the terminal close the window normally
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    # Ask for map index (e.g., 1, 2, 3, 4)
    try:
        index = int(input("Enter mission/map index (e.g., 1, 2, 3, 4): "))
    except ValueError:
        print("Invalid input. Defaulting to index 1.")
        index = 1

    # Load the correct map using an f-string
    costmap_path = Path(f"inputs/costmap_{index}.npz")
    if not costmap_path.exists():
        alt_paths = list(Path("inputs").glob(f"*{index}*.npz"))
        if alt_paths:
            costmap_path = alt_paths[0]
        else:
            print(f"Error: Could not find costmap for index {index} in inputs/")
            return

    with np.load(costmap_path) as data:
        costmap = data["total"].astype(np.float32)

    out_path = Path("outputs")
    
    # Load generation log to extract precise per-map scores if available
    log_path = out_path / "generation_log.json"
    map_scores_lookup = {}
    if log_path.exists():
        with open(log_path, "r") as f:
            log_data = json.load(f)
            map_name = f"map_{index}"
            for item in log_data.get("waypoint_breakdown", []):
                wp_id = item["id"]
                if "scores" in item and map_name in item["scores"]:
                    map_scores_lookup[wp_id] = item["scores"][map_name]

    # Find and sort all waypoint files matching exporter naming format
    wp_files = sorted(
        out_path.glob("wp*.npy"),
        key=lambda p: int(re.match(r"wp(\d+)_", p.name).group(1))
    )

    if len(wp_files) == 0:
        print("No waypoints found to plot in outputs/.")
        return

    # Load each set and assign its score into memory
    waypoints = []
    scores = []
    for wp_file in wp_files:
        match = re.match(r"wp(\d+)_(\d+)\.npy", wp_file.name)
        wp_id = f"wp{int(match.group(1)):02d}"
        
        # Use specific map score if available, otherwise fallback to filename average score
        if wp_id in map_scores_lookup:
            scores.append(round(map_scores_lookup[wp_id], 1))
        else:
            scores.append(int(match.group(2)))
            
        waypoints.append(np.load(wp_file))

    # Set up figure with extra space at the bottom for interactive buttons
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.2)

    # State tracking
    current_idx = [0]

    # Render base costmap
    ax.imshow(costmap, cmap='inferno', origin='lower')
    ax.set_title(f"Map {index} | WP{current_idx[0]:02d} / {len(waypoints) - 1} | Score: {scores[current_idx[0]]}")

    # Initialize plot artists for fast updating
    wp_set = waypoints[current_idx[0]]
    x_coords, y_coords = wp_set[:, 0], wp_set[:, 1]

    line_plot, = ax.plot(x_coords, y_coords, color='cyan', marker='o', linewidth=2.5, markersize=8, label='Trajectory')
    start_plot, = ax.plot([x_coords[0]], [y_coords[0]], color='green', markersize=14, marker='*', linestyle='None', label='Start')
    end_plot, = ax.plot([x_coords[-1]], [y_coords[-1]], color='red', markersize=12, marker='X', linestyle='None', label='End')

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    def update_plot(idx):
        wp_set = waypoints[idx]
        xc, yc = wp_set[:, 0], wp_set[:, 1]

        line_plot.set_xdata(xc)
        line_plot.set_ydata(yc)
        start_plot.set_xdata([xc[0]])
        start_plot.set_ydata([yc[0]])
        end_plot.set_xdata([xc[-1]])
        end_plot.set_ydata([yc[-1]])

        ax.set_title(f"Map {index} | WP{idx:02d} / {len(waypoints) - 1} | Score: {scores[idx]}")
        fig.canvas.draw_idle()

    def next_wp(event):
        current_idx[0] = (current_idx[0] + 1) % len(waypoints)
        update_plot(current_idx[0])

    def prev_wp(event):
        current_idx[0] = (current_idx[0] - 1) % len(waypoints)
        update_plot(current_idx[0])

    def on_key(event):
        if event.key in ['right', 'n']:
            current_idx[0] = (current_idx[0] + 1) % len(waypoints)
            update_plot(current_idx[0])
        elif event.key in ['left', 'p']:
            current_idx[0] = (current_idx[0] - 1) % len(waypoints)
            update_plot(current_idx[0])

    fig.canvas.mpl_connect('key_press_event', on_key)

    ax_prev = plt.axes([0.3, 0.05, 0.15, 0.075])
    ax_next = plt.axes([0.55, 0.05, 0.15, 0.075])

    btn_prev = Button(ax_prev, 'Previous')
    btn_prev.on_clicked(prev_wp)

    btn_next = Button(ax_next, 'Next')
    btn_next.on_clicked(next_wp)

    plt.show()

if __name__ == "__main__":
    plot_interactive_waypoints()