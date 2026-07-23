#!/usr/bin/env python3
import os
import sys
import argparse
import datetime
import xml.etree.ElementTree as ET
import numpy as np

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

def get_base_world_path(world_name="marsyard.world"):
    """Locate base world file from worlds package."""
    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_worlds = get_package_share_directory('worlds')
        world_path = os.path.join(pkg_worlds, 'worlds', world_name)
        if os.path.exists(world_path):
            return world_path
    except Exception:
        pass

    base_paths = [
        os.path.join(get_package_src_dir(), '..', '..', 'marsyards', 'worlds', 'worlds', world_name),
        f'/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds/worlds/{world_name}',
        f'/home/saif/Desktop/ROAR/MARS_YARD_INIT/FinalYard/src/worlds/worlds/{world_name}',
    ]
    for p in base_paths:
        p_abs = os.path.abspath(p)
        if os.path.exists(p_abs):
            return p_abs
    return None

def fuse_obstacle_data_into_world(input_npy_path=None, world_name="marsyard.world", output_world_path=None):
    """
    Fuses numpy obstacle data from package obs_data folder into a Gazebo .world file
    and saves the resulting .world file directly inside src/ package Gen_worlds/ directory.
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

    # Resolve output directory (Gen_worlds inside package src)
    if output_world_path is None:
        gen_worlds_dir = os.path.join(pkg_src_dir, 'Gen_worlds')
        os.makedirs(gen_worlds_dir, exist_ok=True)
        world_base_name = os.path.splitext(world_name)[0]
        output_world_path = os.path.join(gen_worlds_dir, f"{world_base_name}_with_rocks.world")
    else:
        gen_worlds_dir = os.path.dirname(os.path.abspath(output_world_path))
        if gen_worlds_dir:
            os.makedirs(gen_worlds_dir, exist_ok=True)

    obs_data = np.load(input_npy_path, allow_pickle=True)
    if len(obs_data) == 0:
        print("Warning: Obstacle data array is empty.")

    print(f"==================================================")
    print(f"   Fusing Obstacle Data into Gazebo World File")
    print(f"==================================================")
    print(f"  Input Dataset:            {input_npy_path}")
    print(f"  Base World File:          {base_world_file}")
    print(f"  Total Obstacles to Fuse:  {len(obs_data)}")
    print(f"  Package Source Dir:       {pkg_src_dir}")
    print(f"  Output World File:        {output_world_path}")
    print(f"==================================================")

    tree = ET.parse(base_world_file)
    root = tree.getroot()
    world_elem = root.find('world')
    if world_elem is None:
        print("Error: Invalid SDF file, missing <world> root element.", file=sys.stderr)
        return None

    for i, item in enumerate(obs_data, 1):
        r = item if isinstance(item, dict) else (item.item() if hasattr(item, 'item') else dict(item))

        x = float(r.get('x', 0.0))
        y = float(r.get('y', 0.0))
        z = float(r.get('z', 0.0))
        roll = float(r.get('roll', 0.0))
        pitch = float(r.get('pitch', 0.0))
        yaw = float(r.get('yaw', 0.0))
        rock_id = int(r.get('rock_id', 1))
        is_collidable = bool(r.get('is_collidable', True))
        model_name = str(r.get('name', f"Rock_{i}"))

        model_el = ET.SubElement(world_elem, 'model', attrib={'name': model_name})
        
        static_el = ET.SubElement(model_el, 'static')
        static_el.text = 'true'

        pose_el = ET.SubElement(model_el, 'pose')
        pose_el.text = f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"

        link_el = ET.SubElement(model_el, 'link', attrib={'name': 'link'})

        visual_el = ET.SubElement(link_el, 'visual', attrib={'name': 'visual'})
        v_geom_el = ET.SubElement(visual_el, 'geometry')
        v_mesh_el = ET.SubElement(v_geom_el, 'mesh')
        v_uri_el = ET.SubElement(v_mesh_el, 'uri')
        v_uri_el.text = f"package://rock_generator/rocks_ws/rock_{rock_id}/meshes/rock_{rock_id}.stl"

        if is_collidable:
            col_el = ET.SubElement(link_el, 'collision', attrib={'name': 'collision'})
            c_geom_el = ET.SubElement(col_el, 'geometry')
            c_mesh_el = ET.SubElement(c_geom_el, 'mesh')
            c_uri_el = ET.SubElement(c_mesh_el, 'uri')
            c_uri_el.text = f"package://rock_generator/rocks_ws/rock_{rock_id}/meshes/rock_{rock_id}.stl"

    ET.indent(tree, space="  ", level=0)

    # Save primary fused .world file directly in package src Gen_worlds/
    tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
    print(f"-> Saved latest fused world to package src: {output_world_path}")

    # Save timestamped copy in src Gen_worlds/
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    world_base_name = os.path.splitext(world_name)[0]
    timestamped_world_path = os.path.join(gen_worlds_dir, f"{world_base_name}_rocks_{timestamp}.world")
    tree.write(timestamped_world_path, encoding='utf-8', xml_declaration=True)
    print(f"-> Saved timestamped world to package src: {timestamped_world_path}")

    return output_world_path

def parse_args():
    parser = argparse.ArgumentParser(description="Fuse Obstacle Data (.npy) into Gazebo .world file saved in package src Gen_worlds/")
    parser.add_argument("-i", "--input", type=str, default=None,
                        help="Input obstacle_data.npy path (default: obs_data/obstacle_data.npy in package src)")
    parser.add_argument("-w", "--world-name", type=str, default="marsyard.world",
                        help="Base world file name from worlds package (default: marsyard.world)")
    parser.add_argument("-o", "--output", type=str, default=None,
                        help="Output .world file path (default: Gen_worlds/marsyard_with_rocks.world in package src)")
    return parser.parse_args()

def main():
    args = parse_args()
    fuse_obstacle_data_into_world(
        input_npy_path=args.input,
        world_name=args.world_name,
        output_world_path=args.output
    )

if __name__ == '__main__':
    main()
