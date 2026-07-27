import numpy as np
import json

def display_outputs(outputs_dir="outputs"):
    # 1. Load the binary array
    waypoints = np.load(f"{outputs_dir}/waypoints.npy")

    # 2. Load the metadata
    with open(f"{outputs_dir}/generation_log.json", "r") as f:
        log = json.load(f)

    print(f"--- GENERATION LOG ---")
    print(f"Timestamp: {log['timestamp']}")
    print(f"Total Waypoint Sets: {log['wp_count']}\n")

    # 3. Iterate through the waypoint sets (index 0 = wp00 = easiest)
    for i, wp_set in enumerate(waypoints):
        print(f"=== WP{i:02d} ===")
        for j, pt in enumerate(wp_set):
            print(f"  Point {j + 1}: (x: {pt[0]:>3}, y: {pt[1]:>3})")
        print("-" * 40)

if __name__ == "__main__":
    display_outputs()