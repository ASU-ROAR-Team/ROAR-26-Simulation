#!/usr/bin/env python3
import os
import sys
import argparse
import datetime
import copy
import xml.etree.ElementTree as ET
import numpy as np

def get_package_src_dir():
    """Locates the world_gen directory inside navMission_setup/world_setup/."""
    return os.path.dirname(os.path.abspath(__file__))


def get_base_world_path(world_name="marsyard.world"):
    """Locate base world file from direct path or world_setup/initial_inputs/i_world/."""
    if os.path.isfile(world_name):
        return os.path.abspath(world_name)
    if not world_name.endswith('.world'):
        world_name += '.world'

    candidate = os.path.abspath(
        os.path.join(get_package_src_dir(), '..', 'initial_inputs', 'i_world', world_name)
    )
    if os.path.exists(candidate):
        return candidate
    return None

def fuse_obstacle_data_into_world(input_npy_path=None, world_name="marsyard.world", output_world_path=None, density=None, collidable_ratio=None):
    """
    Fuses numpy obstacle data from package obs_data folder into a Gazebo .world file
    and saves the resulting .world file directly inside the worlds package.
    """
    pkg_src_dir = get_package_src_dir()

    # Resolve default input npy file if not provided
    if input_npy_path is None:
        input_npy_path = os.path.join(pkg_src_dir, 'obs_data', 'obstacle_data.npy')

    if not os.path.exists(input_npy_path):
        print(f"Error: Obstacle dataset not found at: {input_npy_path}", file=sys.stderr)
        return None

    # Resolve base world path
    base_world_file = get_base_world_path(world_name)
    if not base_world_file or not os.path.exists(base_world_file):
        print(f"Error: Base world file '{world_name}' not found.", file=sys.stderr)
        return None

    obs_data = np.load(input_npy_path, allow_pickle=True)
    parsed_obs = [
        (r if isinstance(r, dict) else (r.item() if hasattr(r, 'item') else dict(r)))
        for r in obs_data
    ]
    rock_items = [r for r in parsed_obs if int(r.get('mesh_id', r.get('rock_id', 1))) >= 0]
    if len(rock_items) == 0:
        print("Warning: Obstacle data array has no physical rocks.")

    # Calculate density & collidable ratio over physical rocks if not provided
    if density is None:
        # Default sampling bounds cover an area of 704.0 m^2
        density = len(rock_items) / 704.0 if len(rock_items) > 0 else 0.012
    if collidable_ratio is None:
        if len(rock_items) > 0:
            collidables = sum(1 for r in rock_items if r.get('is_collidable', True))
            collidable_ratio = collidables / len(rock_items)
        else:
            collidable_ratio = 0.5

    # Determine output path
    world_filename = f"w_d{density:.3f}_c{collidable_ratio:.2f}.world"

    if output_world_path is None:
        gen_worlds_dir = os.path.join(pkg_src_dir, 'outputs')
        os.makedirs(gen_worlds_dir, exist_ok=True)
        output_world_path = os.path.join(gen_worlds_dir, world_filename)
    else:
        gen_worlds_dir = os.path.dirname(os.path.abspath(output_world_path))
        if gen_worlds_dir:
            os.makedirs(gen_worlds_dir, exist_ok=True)

    print(f"==================================================")
    print(f"   Fusing Obstacle Data into Gazebo World File")
    print(f"==================================================")
    print(f"  Input Dataset:            {input_npy_path}")
    print(f"  Base World File:          {base_world_file}")
    print(f"  Total Obstacles to Fuse:  {len(rock_items)}")
    print(f"  Estimated Density:        {density:.4f}")
    print(f"  Estimated Collidable:     {collidable_ratio:.2f}")
    print(f"  Output World File:        {output_world_path}")
    print(f"==================================================")

    tree = ET.parse(base_world_file)
    root = tree.getroot()
    world_elem = root.find('world')
    if world_elem is None:
        print("Error: Invalid SDF file, missing <world> root element.", file=sys.stderr)
        return None

    for i, r in enumerate(rock_items, 1):
        x = float(r.get('x', 0.0))
        y = float(r.get('y', 0.0))
        z = float(r.get('z', 0.0))
        roll = float(r.get('roll', 0.0))
        pitch = float(r.get('pitch', 0.0))
        yaw = float(r.get('yaw', 0.0))
        rock_id = int(r.get('rock_id', 1))
        mesh_id = int(r.get('mesh_id', rock_id))
        is_collidable = bool(r.get('is_collidable', True))
        model_name = str(r.get('name', f"Rock_{i}"))

        # Inline the selected model so the generated instance can faithfully
        # honor is_collidable.  Visual and collision still use the exact same
        # source mesh and local frame from the rock model template.
        model_dir = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '..', 'rocks_ws', f'rock_{mesh_id}')
        )
        model_sdf = os.path.join(model_dir, 'model.sdf')
        if not os.path.isfile(model_sdf):
            raise FileNotFoundError(f"Rock model template not found: {model_sdf}")
        model_root = ET.parse(model_sdf).getroot()
        template = model_root.find('model') if model_root.tag != 'model' else model_root
        if template is None:
            raise ValueError(f"Rock model template has no <model>: {model_sdf}")

        model_el = copy.deepcopy(template)
        model_el.set('name', model_name)
        old_pose = model_el.find('pose')
        if old_pose is not None:
            model_el.remove(old_pose)
        pose_el = ET.Element('pose')
        pose_el.text = f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"
        
        static_el = model_el.find('static')
        if static_el is None:
            static_el = ET.Element('static')
            model_el.insert(0, static_el)
        static_el.text = 'true'
        
        model_el.insert(1, pose_el)

        if not is_collidable:
            for link_el in model_el.findall('link'):
                for collision_el in list(link_el.findall('collision')):
                    link_el.remove(collision_el)

        world_elem.append(model_el)

    ET.indent(tree, space="  ", level=0)

    # Save primary fused .world file
    tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
    print(f"-> Saved fused world to: {output_world_path}")

    return output_world_path

def parse_args():
    parser = argparse.ArgumentParser(description="Fuse Obstacle Data (.npy) into Gazebo .world file saved in worlds package")
    parser.add_argument("-i", "--input", type=str, default=None,
                        help="Input obstacle_data.npy path (default: obs_data/obstacle_data.npy in package src)")
    parser.add_argument("-w", "--world-name", type=str, default="marsyard.world",
                        help="Base world file name from worlds package (default: marsyard.world)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output world file path (default: generated dynamically inside worlds package)")
    parser.add_argument("--density", type=float, default=None,
                        help="Density value for the output filename")
    parser.add_argument("--collidable-ratio", type=float, default=None,
                        help="Collidable ratio value for the output filename")
    return parser.parse_args()

def main():
    args = parse_args()
    fuse_obstacle_data_into_world(
        input_npy_path=args.input,
        world_name=args.world_name,
        output_world_path=args.output,
        density=args.density,
        collidable_ratio=args.collidable_ratio
    )

if __name__ == '__main__':
    main()
