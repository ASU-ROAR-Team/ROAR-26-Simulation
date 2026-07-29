import numpy as np
import os

# ==========================================
# 1. CONFIGURATION PARAMETERS
# ==========================================
# Offset mapping: x_sim = x_pdf + OFFSET_X, y_sim = y_pdf + OFFSET_Y
# This defines the Gazebo coordinate where the PDF origin (0,0) (S1 corner) lies.
OFFSET_X = -16.0
OFFSET_Y = -6.0

# ==========================================
# 2. LANDMARK PDF COORDINATES (L1 to L15)
# ==========================================
# Edit the 'x' and 'y' values here to update the positions of landmarks.
LANDMARKS = [
    {"name": "ArUco_1",  "x": 3.1374,  "y": 4.3246},
    {"name": "ArUco_2",  "x": 9.0888,  "y": -4.5555},
    {"name": "ArUco_3",  "x": 8.2731,  "y": 2.2478},
    {"name": "ArUco_4",  "x": 13.5552, "y": 3.3260},
    {"name": "ArUco_5",  "x": 17.6623, "y": -2.7646},
    {"name": "ArUco_6",  "x": 23.8746, "y": -2.3014},
    {"name": "ArUco_7",  "x": 27.7097, "y": 2.7192},
    {"name": "ArUco_8",  "x": 28.3320, "y": 8.6813},
    {"name": "ArUco_9",  "x": 25.8693, "y": 7.3461},
    {"name": "ArUco_10", "x": 18.6570, "y": 4.5163},
    {"name": "ArUco_11", "x": 14.9031, "y": 6.1368},
    {"name": "ArUco_12", "x": 13.2623, "y": 11.3769},
    {"name": "ArUco_13", "x": 10.0015, "y": 5.6827},
    {"name": "ArUco_14", "x": 8.0354,  "y": 12.9120},
    {"name": "ArUco_15", "x": 2.7876,  "y": 13.5601}
]

# ==========================================
# 3. INTERPOLATION & EXTRACTION LOGIC
# ==========================================
dir_path = "/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/world_setup/TempArucoGen"
heightmap_path = os.path.join(dir_path, "heightmap", "newhight.npz")
output_npy_path = os.path.join(dir_path, "aruco_data", "aruco_data.npy")

# Load heightmap for Z-height extraction
if not os.path.exists(heightmap_path):
    raise FileNotFoundError(f"Heightmap data not found at {heightmap_path}. Run step1_heightmap.sh first.")

heightmap_data = np.load(heightmap_path)
xs = heightmap_data['xs']
ys = heightmap_data['ys']
grid = heightmap_data['grid']

def get_terrain_height(x, y):
    ix = np.argmin(np.abs(xs - x))
    iy = np.argmin(np.abs(ys - y))
    val = grid[iy, ix]
    if np.isnan(val):
        return 0.2
    return float(val)

# Map coordinates and build shape (15, 3) coordinate array
mapped_coordinates = []
for lm in LANDMARKS:
    x_sim = lm["x"] + OFFSET_X
    y_sim = lm["y"] + OFFSET_Y
    z_sim = get_terrain_height(x_sim, y_sim)
    
    mapped_coordinates.append([x_sim, y_sim, z_sim])

# Convert to numpy array of shape (15, 3)
mapped_array = np.array(mapped_coordinates, dtype=np.float64)

# Save file
np.save(output_npy_path, mapped_array)
print(f"Successfully generated mapped coordinates of shape {mapped_array.shape} at: {output_npy_path}")
for i, coord in enumerate(mapped_array, 1):
    print(f"ArUco_{i}: X={coord[0]:.4f}, Y={coord[1]:.4f}, Z={coord[2]:.4f}")
