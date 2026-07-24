import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

def plot_interactive_missions():
    # Load the map, missions, and difficulty scores
    with np.load("inputs/costmap.npz") as data:
        costmap = data["total"].astype(np.float32)
    
    missions = np.load("outputs/missions.npy")
    scores = np.load("outputs/difficulty.npy")
    
    if len(missions) == 0:
        print("No missions found to plot.")
        return

    # Set up figure with extra space at the bottom for interactive buttons
    fig, ax = plt.subplots(figsize=(10, 8))
    plt.subplots_adjust(bottom=0.2)
    
    # State tracking
    current_idx = [0]  # Mutable container for index tracking
    
    # Render base costmap
    ax.imshow(costmap, cmap='inferno', origin='lower')
    ax.set_title(f"Mission {current_idx[0] + 1} / {len(missions)} | Difficulty Score: {scores[current_idx[0]]:.2f}")
    
    # Initialize plot artists for fast updating
    mission = missions[current_idx[0]]
    x_coords, y_coords = mission[:, 0], mission[:, 1]
    
    line_plot, = ax.plot(x_coords, y_coords, color='cyan', marker='o', linewidth=2.5, markersize=8, label='Trajectory')
    start_plot, = ax.plot([x_coords[0]], [y_coords[0]], color='green', markersize=14, marker='*', linestyle='None', label='Start')
    end_plot, = ax.plot([x_coords[-1]], [y_coords[-1]], color='red', markersize=12, marker='X', linestyle='None', label='End')
    
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')

    def update_plot(index):
        m = missions[index]
        xc, yc = m[:, 0], m[:, 1]
        
        # Update coordinates efficiently without redrawing the whole figure background
        line_plot.set_xdata(xc)
        line_plot.set_ydata(yc)
        start_plot.set_xdata([xc[0]])
        start_plot.set_ydata([yc[0]])
        end_plot.set_xdata([xc[-1]])
        end_plot.set_ydata([yc[-1]])
        
        ax.set_title(f"Mission {index + 1} / {len(missions)} | Difficulty Score: {scores[index]:.2f}")
        fig.canvas.draw_idle()

    # Event Handlers
    def next_mission(event):
        current_idx[0] = (current_idx[0] + 1) % len(missions)
        update_plot(current_idx[0])

    def prev_mission(event):
        current_idx[0] = (current_idx[0] - 1) % len(missions)
        update_plot(current_idx[0])

    def on_key(event):
        if event.key in ['right', 'n']:
            current_idx[0] = (current_idx[0] + 1) % len(missions)
            update_plot(current_idx[0])
        elif event.key in ['left', 'p']:
            current_idx[0] = (current_idx[0] - 1) % len(missions)
            update_plot(current_idx[0])

    fig.canvas.mpl_connect('key_press_event', on_key)

    # Create UI Buttons
    ax_prev = plt.axes([0.3, 0.05, 0.15, 0.075])
    ax_next = plt.axes([0.55, 0.05, 0.15, 0.075])
    
    btn_prev = Button(ax_prev, 'Previous')
    btn_prev.on_clicked(prev_mission)
    
    btn_next = Button(ax_next, 'Next')
    btn_next.on_clicked(next_mission)

    plt.show()

if __name__ == "__main__":
    plot_interactive_missions()