#!/usr/bin/env python3
import os
import sys
import argparse
import datetime
import xml.etree.ElementTree as ET
import numpy as np

def get_package_src_dir():
    """Locates the source directory of the rock_generator package inside src/."""
    # Prioritize correct source workspace directory structure
    hardcoded = '/home/saif/Desktop/ROAR/simulation_ws/src/navMission_setup/rock_generator'
    if os.path.exists(os.path.join(hardcoded, 'package.xml')):
        return hardcoded

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
    if not world_name.endswith('.world'):
        world_name += '.world'
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


def get_worlds_package_src_dir():
    """Locates the source directory of the worlds package inside src/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('worlds')
        parts = share_dir.split(os.sep)
        if 'install' in parts:
            idx = parts.index('install')
            ws_root = os.sep.join(parts[:idx])
            target_src = os.path.join(ws_root, 'src', 'marsyards', 'worlds')
            if os.path.exists(os.path.join(target_src, 'package.xml')):
                return target_src
    except Exception:
        pass
    # Fallback to hardcoded path
    fallback = '/home/saif/Desktop/ROAR/simulation_ws/src/marsyards/worlds'
    if os.path.exists(fallback):
        return fallback
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
    if len(obs_data) == 0:
        print("Warning: Obstacle data array is empty.")

    # Calculate density & collidable ratio if not provided
    if density is None:
        # Default sampling bounds cover an area of 704.0 m^2
        density = len(obs_data) / 704.0 if len(obs_data) > 0 else 0.012
    if collidable_ratio is None:
        if len(obs_data) > 0:
            collidables = sum(1 for r in obs_data if (r if isinstance(r, dict) else (r.item() if hasattr(r, 'item') else dict(r))).get('is_collidable', True))
            collidable_ratio = collidables / len(obs_data)
        else:
            collidable_ratio = 0.5

    # Determine output path
    worlds_src_dir = get_worlds_package_src_dir()
    world_filename = f"w_d{density:.3f}_c{collidable_ratio:.2f}.world"

    if output_world_path is None:
        if worlds_src_dir:
            worlds_dir = os.path.join(worlds_src_dir, 'worlds')
            os.makedirs(worlds_dir, exist_ok=True)
            output_world_path = os.path.join(worlds_dir, world_filename)
        else:
            gen_worlds_dir = os.path.join(pkg_src_dir, 'Gen_worlds')
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
    print(f"  Total Obstacles to Fuse:  {len(obs_data)}")
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

    for i, item in enumerate(obs_data, 1):
        r = item if isinstance(item, dict) else (item.item() if hasattr(item, 'item') else dict(item))

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
        v_uri_el.text = f"package://rock_generator/rocks_ws/rock_{mesh_id}/meshes/rock_{mesh_id}.obj"

        if is_collidable:
            col_el = ET.SubElement(link_el, 'collision', attrib={'name': 'collision'})
            c_geom_el = ET.SubElement(col_el, 'geometry')
            c_mesh_el = ET.SubElement(c_geom_el, 'mesh')
            c_uri_el = ET.SubElement(c_mesh_el, 'uri')
            c_uri_el.text = f"package://rock_generator/rocks_ws/rock_{mesh_id}/meshes/rock_{mesh_id}.obj"

    ET.indent(tree, space="  ", level=0)

    # Save primary fused .world file
    tree.write(output_world_path, encoding='utf-8', xml_declaration=True)
    print(f"-> Saved fused world to: {output_world_path}")

    # Generate a launch file inside the worlds package launch directory
    if worlds_src_dir:
        launch_filename = f"w_d{density:.3f}_c{collidable_ratio:.2f}.launch.py"
        launch_path = os.path.join(worlds_src_dir, 'launch', launch_filename)
        
        launch_content = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_worlds = get_package_share_directory('worlds')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_worlds, 'launch', 'launch_map.launch.py')
            ),
            launch_arguments={{'world': '{world_filename}'}}.items()
        )
    ])
"""
        try:
            with open(launch_path, 'w') as lf:
                lf.write(launch_content)
            print(f"-> Generated launch file: {launch_path}")
        except Exception as e:
            print(f"Warning: Failed to generate launch file at {launch_path}: {e}")

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

