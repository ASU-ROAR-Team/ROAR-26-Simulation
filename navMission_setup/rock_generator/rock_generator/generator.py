#!/usr/bin/env python3
import os
import sys
import math
import random
import argparse
import datetime
import numpy as np

# Bounding box of the visual mesh terrain map
VISUAL_X_MIN, VISUAL_X_MAX = -21.5503, 21.6497
VISUAL_Y_MIN, VISUAL_Y_MAX = -14.7658, 14.7841
VISUAL_X_LEN = VISUAL_X_MAX - VISUAL_X_MIN
VISUAL_Y_LEN = VISUAL_Y_MAX - VISUAL_Y_MIN

def get_package_src_dir():
    """Locates the source directory of the rock_generator package inside src/."""
    mod_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_src = os.path.dirname(mod_dir)
    if os.path.exists(os.path.join(pkg_src, 'package.xml')):
        return pkg_src

    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('rock_generator')
        parts = share_dir.split(os.sep)
        if 'install' in parts:
            idx = parts.index('install')
            ws_root = os.sep.join(parts[:idx])
            target_src = os.path.join(ws_root, 'src', 'navMission_setup', 'rock_generator')
            if os.path.exists(os.path.join(target_src, 'package.xml')):
                return target_src
    except Exception:
        pass

    return pkg_src

def get_mask_path():
    """Locate marsyard_exact_mask_closed.png dynamically."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('marsyard')
        path = os.path.join(pkg_share, 'maps', 'marsyard_exact_mask_closed.png')
        if os.path.exists(path):
            return path
    except Exception:
        pass

    base_paths = [
        os.path.join(get_package_src_dir(), '..', '..', 'marsyards', 'marsyard', 'maps', 'marsyard_exact_mask_closed.png'),
        '/home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard/src/marsyard/maps/marsyard_exact_mask_closed.png',
        '/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/marsyard/maps/marsyard_exact_mask_closed.png',
    ]
    for p in base_paths:
        p_abs = os.path.abspath(p)
        if os.path.exists(p_abs):
            return p_abs
    return None

def is_on_ground(x, y, mask_path=None):
    """Checks if coordinate (x, y) is on active ground terrain mask."""
    if not mask_path:
        mask_path = get_mask_path()
        
    if not mask_path or not os.path.exists(mask_path):
        return -20.0 <= x <= 13.0 and -13.0 <= y <= 12.0
        
    try:
        from PIL import Image
        img = Image.open(mask_path).convert('L')
        w, h = img.size
        
        px = int(((x - VISUAL_X_MIN) / VISUAL_X_LEN) * w)
        py = int(((y - VISUAL_Y_MIN) / VISUAL_Y_LEN) * h)
        
        px = max(0, min(w - 1, px))
        py = max(0, min(h - 1, py))
        
        return img.getpixel((px, py)) > 127
    except Exception as e:
        return -20.0 <= x <= 13.0 and -13.0 <= y <= 12.0

def generate_obstacle_data(
    world_name="marsyard.world",
    density=0.012,
    collidable_ratio=0.5,
    spacing=1.0,
    x_range=(-21.5, 21.5),
    y_range=(-14.7, 14.7),
    spawn_z=4.0,
    deadends=False,
    output_file=None
):
    """
    Generates rock obstacle configuration and saves as numpy (.npy) format file directly inside src/ package folder.
    """
    pkg_src_dir = get_package_src_dir()
    
    if output_file is None:
        output_dir = os.path.join(pkg_src_dir, 'obs_data')
        os.makedirs(output_dir, exist_ok=True)
        output_file = os.path.join(output_dir, 'obstacle_data.npy')
    else:
        output_dir = os.path.dirname(os.path.abspath(output_file))
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

    area = (x_range[1] - x_range[0]) * (y_range[1] - y_range[0])
    num_rocks = max(1, int(round(density * area)))
    
    print(f"==================================================")
    print(f"   Generating Obstacle Data File (.npy)")
    print(f"==================================================")
    print(f"  Target World:             {world_name}")
    print(f"  Calculated Area:          {area:.1f} m²")
    print(f"  Rock Density:             {density} rocks/m²")
    print(f"  Total Rocks:              {num_rocks}")
    print(f"  Collidable Ratio:         {collidable_ratio:.2f}")
    print(f"  Rock Spacing Min:         {spacing:.2f} m")
    print(f"  Bounds:                   X={x_range}, Y={y_range}")
    print(f"  Package Source Dir:       {pkg_src_dir}")
    print(f"  Primary Output File:      {output_file}")
    print(f"==================================================")

    barrier_coords = []
    if deadends:
        xs = [-3.0, -1.5, 0.0, 1.5, 3.0]
        num_barrier = min(num_rocks, len(xs))
        barrier_coords = [(xs[i], 2.5) for i in range(num_barrier)]
        
    mask_path = get_mask_path()
    placed_coords = []
    rock_data_list = []

    # Place barrier rocks
    for idx, (x, y) in enumerate(barrier_coords, 1):
        yaw = random.uniform(-math.pi, math.pi)
        rock_id = random.randint(1, 5)
        rock_entry = {
            'id': idx,
            'name': f"Rock_{idx}",
            'x': float(x),
            'y': float(y),
            'z': float(spawn_z),
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': float(yaw),
            'rock_id': int(rock_id),
            'is_collidable': True,
            'is_barrier': True,
            'world_name': str(world_name)
        }
        placed_coords.append((x, y))
        rock_data_list.append(rock_entry)

    # Place random rocks with min spacing enforcement
    for idx in range(len(barrier_coords) + 1, num_rocks + 1):
        attempts = 0
        max_attempts = 1000
        best_x, best_y = None, None
        
        while attempts < max_attempts:
            cand_x = random.uniform(x_range[0], x_range[1])
            cand_y = random.uniform(y_range[0], y_range[1])
            attempts += 1
            
            if not is_on_ground(cand_x, cand_y, mask_path):
                continue
                
            too_close = False
            for (px, py) in placed_coords:
                dist = math.hypot(cand_x - px, cand_y - py)
                if dist < spacing:
                    too_close = True
                    break
            
            if not too_close:
                best_x, best_y = cand_x, cand_y
                break

        if best_x is None:
            while True:
                cand_x = random.uniform(x_range[0], x_range[1])
                cand_y = random.uniform(y_range[0], y_range[1])
                if is_on_ground(cand_x, cand_y, mask_path):
                    best_x, best_y = cand_x, cand_y
                    break

        yaw = random.uniform(-math.pi, math.pi)
        rock_id = random.randint(1, 9)
        is_collidable = (random.random() < collidable_ratio)
        
        rock_entry = {
            'id': idx,
            'name': f"Rock_{idx}",
            'x': float(best_x),
            'y': float(best_y),
            'z': float(spawn_z),
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': float(yaw),
            'rock_id': int(rock_id),
            'is_collidable': bool(is_collidable),
            'is_barrier': False,
            'world_name': str(world_name)
        }
        placed_coords.append((best_x, best_y))
        rock_data_list.append(rock_entry)

    obs_array = np.array(rock_data_list, dtype=object)
    
    # Save primary output file (in src/ directory)
    np.save(output_file, obs_array)
    print(f"-> Saved latest obstacle entries to package src: {output_file}")
    
    # Save a timestamped copy in src obs_data folder so each run creates a new file
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    timestamped_file = os.path.join(output_dir, f"obstacle_data_{timestamp}.npy")
    np.save(timestamped_file, obs_array)
    print(f"-> Saved timestamped dataset to package src: {timestamped_file}")

    # Save readable info file
    info_file = os.path.splitext(output_file)[0] + "_info.txt"
    with open(info_file, 'w') as f:
        f.write(f"Obstacle Data Summary\n")
        f.write(f"World Name: {world_name}\n")
        f.write(f"Total Rocks: {len(rock_data_list)}\n")
        f.write(f"Density: {density} rocks/m²\n")
        f.write(f"Collidable Ratio: {collidable_ratio}\n")
        f.write(f"Spacing Min: {spacing} m\n")
        f.write(f"Deadends: {deadends}\n")
        f.write(f"Timestamped File: obstacle_data_{timestamp}.npy\n\n")
        for r in rock_data_list:
            f.write(f"[{r['id']}] {r['name']} | Rock Model #{r['rock_id']} | Pos: ({r['x']:.2f}, {r['y']:.2f}, {r['z']:.2f}) | Collidable: {r['is_collidable']}\n")

    return output_file

def parse_args():
    parser = argparse.ArgumentParser(description="Generate Obstacle Data (.npy) for ROAR Simulation")
    parser.add_argument("--world-name", type=str, default="marsyard.world",
                        help="Target world name from worlds package (default: marsyard.world)")
    parser.add_argument("--density", type=float, default=0.012,
                        help="Rock density in rocks per square meter (default: 0.012)")
    parser.add_argument("-c", "--collidable-ratio", type=float, default=0.5,
                        help="Ratio of rocks to be collidable (default: 0.5)")
    parser.add_argument("-s", "--spacing", type=float, default=1.0,
                        help="Minimum spacing between rocks in meters (default: 1.0)")
    parser.add_argument("--x-range", type=float, nargs=2, default=[-21.5, 21.5],
                        help="X boundary min max (default: -21.5 21.5)")
    parser.add_argument("--y-range", type=float, nargs=2, default=[-14.7, 14.7],
                        help="Y boundary min max (default: -14.7 14.7)")
    parser.add_argument("--deadends", action="store_true", default=False,
                        help="Generate deadend barrier rock formation")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output .npy file path (default: obs_data/obstacle_data.npy in src package)")
    return parser.parse_args()

def main():
    args = parse_args()
    generate_obstacle_data(
        world_name=args.world_name,
        density=args.density,
        collidable_ratio=args.collidable_ratio,
        spacing=args.spacing,
        x_range=tuple(args.x_range),
        y_range=tuple(args.y_range),
        deadends=args.deadends,
        output_file=args.output
    )

if __name__ == '__main__':
    main()
