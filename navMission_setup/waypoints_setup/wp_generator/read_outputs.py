import numpy as np
import json

def display_outputs(outputs_dir="outputs"):
    # 1. Load the binary arrays
    missions = np.load(f"{outputs_dir}/missions.npy")
    scores = np.load(f"{outputs_dir}/difficulty.npy")
    
    # 2. Load the metadata
    with open(f"{outputs_dir}/generation_log.json", "r") as f:
        log = json.load(f)
        
    print(f"--- GENERATION LOG ---")
    print(f"Timestamp: {log['timestamp']}")
    print(f"Total Missions: {log['mission_count']}\n")

    # 3. Iterate through the "sets"
    for i, (mission, score) in enumerate(zip(missions, scores)):
        print(f"=== MISSION SET {i + 1} | Difficulty Score: {score:.2f} ===")
        for j, wp in enumerate(mission):
            print(f"  WP {j + 1}: (x: {wp[0]:>3}, y: {wp[1]:>3})")
        print("-" * 40)

if __name__ == "__main__":
    display_outputs()
