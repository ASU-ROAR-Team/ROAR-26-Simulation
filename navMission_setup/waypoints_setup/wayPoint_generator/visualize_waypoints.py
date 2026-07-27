import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import re
import signal
from pathlib import Path

signal.signal(signal.SIGINT, signal.SIG_DFL)


def plot_interactive_waypoints():
    # Let Ctrl+C in the terminal close the window normally
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    # Load the map
    with np.load("inputs/costmap.npz") as data:
        costmap = data["total"].astype(np.float32)

    # Find and sort all waypoint files by their index (wp00, wp01, ...)
    out_path = Path("outputs")
    wp_files = sorted(
        out_path.glob("wp*_*.npy"),
        key=lambda p: int(re.match(r"wp(\d+)_", p.name).group(1))
    )

    if len(wp_files) == 0:
        print("No waypoints found to plot.")
        return

    # Load each set and its score into memory once
    waypoints = []
    scores = []
    for wp_file in wp_files:
        match = re.match(r"wp(\d+)_(\d+)\.npy", wp_file.name)
        scores.append(int(match.group(2)))
        waypoints.append(np.load(wp_file))

    # Set up figure with extra space at the bottom for interactive buttons
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.2)

    # State tracking
    current_idx = [0]  # Mutable container for index tracking

    # Render base costmap
    ax.imshow(costmap, cmap='inferno', origin='lower')
    ax.set_title(f"WP{current_idx[0]:02d} / {len(waypoints) - 1} | Score: {scores[current_idx[0]]}")

    # Initialize plot artists for fast updating
    wp_set = waypoints[current_idx[0]]
    x_coords, y_coords = wp_set[:, 0], wp_set[:, 1]

    line_plot, = ax.plot(x_coords, y_coords, color='cyan', marker='o', linewidth=2.5, markersize=8, label='Trajectory')
    start_plot, = ax.plot([x_coords[0]], [y_coords[0]], color='green', markersize=14, marker='*', linestyle='None', label='Start')
    end_plot, = ax.plot([x_coords[-1]], [y_coords[-1]], color='red', markersize=12, marker='X', linestyle='None', label='End')

    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    def update_plot(index):
        wp_set = waypoints[index]
        xc, yc = wp_set[:, 0], wp_set[:, 1]

        line_plot.set_xdata(xc)
        line_plot.set_ydata(yc)
        start_plot.set_xdata([xc[0]])
        start_plot.set_ydata([yc[0]])
        end_plot.set_xdata([xc[-1]])
        end_plot.set_ydata([yc[-1]])

        ax.set_title(f"WP{index:02d} / {len(waypoints) - 1} | Score: {scores[index]}")
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