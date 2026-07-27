import numpy as np
import json
import re
from pathlib import Path

def display_outputs(outputs_dir="outputs"):
    out_path = Path(outputs_dir)

    # 1. Find and sort all waypoint files by their index (wp00, wp01, ...)
    wp_files = sorted(
        out_path.glob("wp*_*.npy"),
        key=lambda p: int(re.match(r"wp(\d+)_", p.name).group(1))
    )

    # 2. Load the metadata
    with open(out_path / "generation_log.json", "r") as f:
        log = json.load(f)

    print(f"--- GENERATION LOG ---")
    print(f"Timestamp: {log['timestamp']}")
    print(f"Total Waypoint Sets: {log['wp_count']}\n")

    # 3. Iterate through the waypoint files (index 0 = wp00 = easiest)
    for wp_file in wp_files:
        match = re.match(r"wp(\d+)_(\d+)\.npy", wp_file.name)
        index, score = int(match.group(1)), int(match.group(2))
        wp_set = np.load(wp_file)

        print(f"=== WP{index:02d} | Score: {score} ===")
        for j, pt in enumerate(wp_set):
            print(f"  Point {j + 1}: (x: {pt[0]:>3}, y: {pt[1]:>3})")
        print("-" * 40)

if __name__ == "__main__":
    display_outputs()