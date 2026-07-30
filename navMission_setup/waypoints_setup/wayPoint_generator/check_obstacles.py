import numpy as np
from pathlib import Path

def inspect_obstacle_data():
    for i in range(1, 4):
        file_path = Path(f"inputs/obstacle_data_{i}.npy")
        
        if not file_path.exists():
            print(f"[WARNING] {file_path} not found.")
            continue
            
        try:
            # Added allow_pickle=True here!
            data = np.load(file_path, allow_pickle=True)
            print(f"--- {file_path.name} ---")
            print(f"Shape:      {data.shape}")
            print(f"Data Type:  {data.dtype}")
            print(f"First 3 rows (Sample):")
            print(data[:3])
            print("-" * 30 + "\n")
        except Exception as e:
            print(f"[ERROR] Could not read {file_path.name}: {e}")

if __name__ == "__main__":
    inspect_obstacle_data()