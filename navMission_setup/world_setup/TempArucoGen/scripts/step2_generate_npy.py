#!/usr/bin/env python3
"""
Generate ArUco Marker Data (.npy, .yaml, .txt) with 3D positions, quaternions,
dimensions, and frame_id.
"""
import os
import math
import numpy as np

# ==========================================
# 1. CONFIGURATION PARAMETERS
# ==========================================
# Offset mapping: x_sim = x_pdf + OFFSET_X, y_sim = y_pdf + OFFSET_Y
OFFSET_X = -16.0
OFFSET_Y = -6.0

DEFAULT_SIZE_M = 0.20
DEFAULT_FRAME_ID = "map"

# ==========================================
# 2. LANDMARK PDF COORDINATES (L1 to L15)
# ==========================================
LANDMARKS = [
    {"id": 1,  "name": "ArUco_1",  "x": 3.1374,  "y": 4.3246,  "yaw": 0.0},
    {"id": 2,  "name": "ArUco_2",  "x": 9.0888,  "y": -4.5555, "yaw": 0.0},
    {"id": 3,  "name": "ArUco_3",  "x": 8.2731,  "y": 2.2478,  "yaw": 0.0},
    {"id": 4,  "name": "ArUco_4",  "x": 13.5552, "y": 3.3260,  "yaw": 0.0},
    {"id": 5,  "name": "ArUco_5",  "x": 17.6623, "y": -2.7646, "yaw": 0.0},
    {"id": 6,  "name": "ArUco_6",  "x": 23.8746, "y": -2.3014, "yaw": 0.0},
    {"id": 7,  "name": "ArUco_7",  "x": 27.7097, "y": 2.7192,  "yaw": 0.0},
    {"id": 8,  "name": "ArUco_8",  "x": 28.3320, "y": 8.6813,  "yaw": 0.0},
    {"id": 9,  "name": "ArUco_9",  "x": 25.8693, "y": 7.3461,  "yaw": 0.0},
    {"id": 10, "name": "ArUco_10", "x": 18.6570, "y": 4.5163,  "yaw": 0.0},
    {"id": 11, "name": "ArUco_11", "x": 14.9031, "y": 6.1368,  "yaw": 0.0},
    {"id": 12, "name": "ArUco_12", "x": 13.2623, "y": 11.3769, "yaw": 0.0},
    {"id": 13, "name": "ArUco_13", "x": 10.0015, "y": 5.6827,  "yaw": 0.0},
    {"id": 14, "name": "ArUco_14", "x": 8.0354,  "y": 12.9120, "yaw": 0.0},
    {"id": 15, "name": "ArUco_15", "x": 2.7876,  "y": 13.5601, "yaw": 0.0}
]


def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """Convert Roll, Pitch, Yaw (radians) to Quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return {
        "x": round(float(qx), 4),
        "y": round(float(qy), 4),
        "z": round(float(qz), 4),
        "w": round(float(qw), 4),
    }


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    temp_dir = os.path.dirname(script_dir)
    heightmap_path = os.path.join(temp_dir, "heightmap", "newhight.npz")
    output_dir = os.path.join(temp_dir, "aruco_data")
    os.makedirs(output_dir, exist_ok=True)

    output_npy_path = os.path.join(output_dir, "aruco_data.npy")
    output_yaml_path = os.path.join(output_dir, "aruco_data.yaml")
    output_info_path = os.path.join(output_dir, "aruco_data_info.txt")

    # Load heightmap if available
    xs, ys, grid = None, None, None
    if os.path.exists(heightmap_path):
        heightmap_data = np.load(heightmap_path)
        xs = heightmap_data['xs']
        ys = heightmap_data['ys']
        grid = heightmap_data['grid']

    def get_terrain_height(x, y):
        if xs is None or ys is None or grid is None:
            return 0.2
        ix = np.argmin(np.abs(xs - x))
        iy = np.argmin(np.abs(ys - y))
        val = grid[iy, ix]
        if np.isnan(val):
            return 0.2
        return float(val)

    marker_list = []
    for lm in LANDMARKS:
        marker_id = lm["id"]
        x_sim = lm["x"] + OFFSET_X
        y_sim = lm["y"] + OFFSET_Y
        z_sim = get_terrain_height(x_sim, y_sim)
        yaw = lm.get("yaw", 0.0)

        quat = euler_to_quaternion(0.0, 0.0, yaw)

        marker_entry = {
            "id": int(marker_id),
            "name": lm["name"],
            "position": {
                "x": round(float(x_sim), 4),
                "y": round(float(y_sim), 4),
                "z": round(float(z_sim), 4),
            },
            "orientation": quat,
            "size_m": DEFAULT_SIZE_M,
            "frame_id": DEFAULT_FRAME_ID,
            # Flat attributes matching rocks schema for perception
            "x": round(float(x_sim), 4),
            "y": round(float(y_sim), 4),
            "z": round(float(z_sim), 4),
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": round(float(yaw), 4),
            "length": DEFAULT_SIZE_M,
            "width": DEFAULT_SIZE_M,
            "height": 0.001,
        }
        marker_list.append(marker_entry)

    # Output structure with top-level 'markers' key
    dataset_structure = {"markers": marker_list}

    # Save .npy file
    np.save(output_npy_path, np.array(dataset_structure, dtype=object))
    print(f"[ArUco Gen] Saved .npy dataset to: {output_npy_path}")

    # Save .yaml file matching requested format
    with open(output_yaml_path, "w") as f:
        f.write("markers:\n")
        for m in marker_list:
            f.write(f"  - id: {m['id']}\n")
            f.write("    position:\n")
            f.write(f"      x: {m['position']['x']:.4f}\n")
            f.write(f"      y: {m['position']['y']:.4f}\n")
            f.write(f"      z: {m['position']['z']:.4f}\n")
            f.write("    orientation:\n")
            f.write(f"      x: {m['orientation']['x']:.4f}\n")
            f.write(f"      y: {m['orientation']['y']:.4f}\n")
            f.write(f"      z: {m['orientation']['z']:.4f}\n")
            f.write(f"      w: {m['orientation']['w']:.4f}\n")
            f.write(f"    size_m: {m['size_m']:.2f}\n")
            f.write(f"    frame_id: {m['frame_id']}\n")
    print(f"[ArUco Gen] Saved .yaml dataset to: {output_yaml_path}")

    # Save .txt summary
    with open(output_info_path, "w") as f:
        f.write("ArUco Marker Summary\n")
        f.write("==================================================\n")
        f.write(f"Total Markers : {len(marker_list)}\n")
        f.write(f"Frame ID      : {DEFAULT_FRAME_ID}\n")
        f.write(f"Marker Size   : {DEFAULT_SIZE_M} m\n\n")
        for m in marker_list:
            p = m["position"]
            f.write(
                f"[{m['id']:2d}] {m['name']:<10} | Pos: ({p['x']:7.4f}, {p['y']:7.4f}, {p['z']:7.4f}) | "
                f"Size: {m['size_m']:.2f} m | Frame: {m['frame_id']}\n"
            )
    print(f"[ArUco Gen] Saved summary text to: {output_info_path}")


if __name__ == "__main__":
    main()
