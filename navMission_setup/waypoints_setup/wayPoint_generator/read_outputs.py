import argparse
import json
import numpy as np
import re
from pathlib import Path
import sys

# Import the interactive visualizer if available
try:
    from visualize_waypoints import plot_interactive_waypoints
except Exception:
    plot_interactive_waypoints = None


def list_waypoint_files(outputs_dir: Path):
    """Return sorted list of waypoint *.npy files in outputs directory.
    Files are expected to follow the pattern wpXX_YY.npy where XX is mission index.
    """
    wp_files = sorted(
        outputs_dir.glob('wp*.npy'),
        key=lambda p: int(re.match(r'wp(\d+)_', p.name).group(1))
    )
    return wp_files


def load_waypoints(wp_file: Path):
    """Load a single waypoint file and return a numpy array of shape (N, 2)."""
    try:
        return np.load(wp_file)
    except Exception as e:
        print(f"Failed to load {wp_file.name}: {e}")
        return None


def summarize_outputs(outputs_dir: Path):
    wp_files = list_waypoint_files(outputs_dir)
    if not wp_files:
        print("No waypoint files found in the outputs directory.")
        return

    total_missions = len(wp_files)
    print(f"Found {total_missions} waypoint missions in '{outputs_dir}'.")
    for idx, wp_file in enumerate(wp_files):
        wp = load_waypoints(wp_file)
        if wp is None:
            continue
        num_pts = wp.shape[0]
        print(f"  [{idx:02d}] {wp_file.name}: {num_pts} waypoints")


def visualize_mission(outputs_dir: Path, mission_idx: int):
    wp_files = list_waypoint_files(outputs_dir)
    if mission_idx < 0 or mission_idx >= len(wp_files):
        print(f"Mission index {mission_idx} out of range (0-{len(wp_files)-1}).")
        return
    wp_file = wp_files[mission_idx]
    wp = load_waypoints(wp_file)
    if wp is None:
        return
    import matplotlib.pyplot as plt
    plt.figure(figsize=(8, 6))
    plt.title(f"Mission {mission_idx:02d}: {wp_file.name}")
    plt.plot(wp[:, 0], wp[:, 1], 'o-c')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="Read and summarize waypoint outputs.")
    parser.add_argument('--outputs', type=str, default='outputs', help='Directory containing generated waypoint .npy files.')
    parser.add_argument('--list', action='store_true', help='List all waypoint files and a brief summary.')
    parser.add_argument('--visualize', type=int, metavar='IDX', help='Visualize the mission with the given index (static plot).')
    parser.add_argument('--interactive', action='store_true', help='Launch the interactive visualizer from visualize_waypoints.py (if available).')
    args = parser.parse_args()

    outputs_dir = Path(args.outputs)
    if not outputs_dir.is_dir():
        print(f"Outputs directory '{outputs_dir}' does not exist.")
        sys.exit(1)

    if args.list:
        summarize_outputs(outputs_dir)
    if args.visualize is not None:
        visualize_mission(outputs_dir, args.visualize)
    if args.interactive:
        if plot_interactive_waypoints is None:
            print("Interactive visualizer not available (missing visualize_waypoints module).")
        else:
            plot_interactive_waypoints()

if __name__ == '__main__':
    main()