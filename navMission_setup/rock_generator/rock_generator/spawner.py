#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import datetime
import xml.etree.ElementTree as ET
import argparse
import numpy as np

FALL_WAIT_TIME = 2.0
SPAWN_DELAY = 1.0
FREEZE_DELAY = 0.3

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

def get_rocks_ws_path():
    """Locate package rocks_ws directory containing model definitions."""
    pkg_src = get_package_src_dir()
    local_rocks = os.path.join(pkg_src, 'rocks_ws')
    if os.path.isdir(local_rocks):
        return local_rocks

    try:
        from ament_index_python.packages import get_package_share_directory
        pkg_share = get_package_share_directory('rock_generator')
        path = os.path.join(pkg_share, 'rocks_ws')
        if os.path.isdir(path):
            return path
    except Exception:
        pass
        
    return None

def get_active_world_name(fallback=None):
    """Detects active Gazebo world name from ign/gz service list."""
    for cli in ["ign", "gz"]:
        try:
            out = subprocess.check_output(f"{cli} service --list", shell=True, text=True, stderr=subprocess.DEVNULL, timeout=5)
            for line in out.splitlines():
                if "/create" in line:
                    parts = line.strip().split('/')
                    if len(parts) >= 3:
                        return parts[2]
        except Exception:
            pass
    return fallback

def wait_for_gazebo(max_wait=30):
    """Waits until Gazebo simulation service becomes active."""
    print("Waiting for active Gazebo simulation service...")
    start_time = time.time()
    while time.time() - start_time < max_wait:
        w_name = get_active_world_name(fallback=None)
        if w_name is not None:
            print(f"-> Active Gazebo world detected: '{w_name}'")
            return w_name
        time.sleep(1.5)
    print("Warning: Gazebo service not detected within timeout. Proceeding with fallback world name 'marsyard'.")
    return "marsyard"

def get_model_pose(name):
    """Retrieves current pose of model from Gazebo."""
    for cli in ["ign", "gz"]:
        try:
            cmd = f'{cli} model -m "{name}" --pose'
            out = subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.DEVNULL, timeout=5)
            import re
            brackets = re.findall(r'\[([^\]]+)\]', out)
            if len(brackets) >= 3:
                xyz = [float(v) for v in brackets[-2].split()]
                rpy = [float(v) for v in brackets[-1].split()]
                if len(xyz) == 3 and len(rpy) == 3:
                    return xyz + rpy
            parts = out.strip().split()
            if len(parts) == 6:
                return [float(val) for val in parts]
        except Exception:
            pass
    return None

def remove_model(world, name):
    """Removes model entity from Gazebo world."""
    for cli in ["ign", "gz"]:
        try:
            cmd = f"{cli} service -s /world/{world}/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 2000 --req 'name: \"{name}\", type: 2'"
            subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=5)
        except Exception:
            pass

def generate_modified_sdf(original_sdf, temp_sdf, model_name, is_static, keep_collision):
    """Modifies SDF file for static/dynamic state and collidability."""
    try:
        tree = ET.parse(original_sdf)
        root = tree.getroot()
        model = root.find('model')
        if model is None:
            return False
            
        model.set('name', model_name)
        static_elem = model.find('static')
        if static_elem is not None:
            static_elem.text = 'true' if is_static else 'false'
        else:
            static_elem = ET.SubElement(model, 'static')
            static_elem.text = 'true' if is_static else 'false'
            
        link = model.find('link')
        if link is not None:
            if not is_static:
                inertial = link.find('inertial')
                if inertial is None:
                    inertial = ET.SubElement(link, 'inertial')
                    mass = ET.SubElement(inertial, 'mass')
                    mass.text = '5.0'
                    inertia = ET.SubElement(inertial, 'inertia')
                    for axis in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']:
                        ax = ET.SubElement(inertia, axis)
                        ax.text = '0.1' if axis in ['ixx', 'iyy', 'izz'] else '0.0'
            if not keep_collision:
                collision = link.find('collision')
                if collision is not None:
                    link.remove(collision)
                    
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)
        return True
    except Exception as e:
        print(f"Error modifying SDF: {e}")
        return False

def spawn_model_cmd(sdf_path, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Calls ros2 run ros_gz_sim create to spawn model into Gazebo."""
    cmd = [
        "ros2", "run", "ros_gz_sim", "create",
        "-file", sdf_path,
        "-name", name,
        "-x", f"{x:.4f}",
        "-y", f"{y:.4f}",
        "-z", f"{z:.4f}",
        "-R", f"{roll:.4f}",
        "-P", f"{pitch:.4f}",
        "-Y", f"{yaw:.4f}"
    ]
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
    except subprocess.TimeoutExpired:
        print(f"Warning: Spawning model '{name}' timed out after 15s.")

def spawn_rocks_from_npy(input_npy_path=None, target_world=None):
    """Loads obstacle_data.npy, spawns/visualizes rocks, and saves final settled positions dataset."""
    pkg_src_dir = get_package_src_dir()

    if input_npy_path is None:
        input_npy_path = os.path.join(pkg_src_dir, 'obs_data', 'obstacle_data.npy')

    if not os.path.exists(input_npy_path):
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('rock_generator')
            input_npy_path = os.path.join(pkg_share, 'obs_data', 'obstacle_data.npy')
        except Exception:
            pass

    if not os.path.exists(input_npy_path):
        print(f"Error: Obstacle data file not found at: {input_npy_path}", file=sys.stderr)
        return False

    rocks_ws = get_rocks_ws_path()
    if not rocks_ws or not os.path.isdir(rocks_ws):
        print(f"Error: Could not locate rocks_ws directory", file=sys.stderr)
        return False

    obs_data = np.load(input_npy_path, allow_pickle=True)
    if len(obs_data) == 0:
        print("Warning: Obstacle data file is empty!")
        return True

    detected_world = wait_for_gazebo(max_wait=30)
    world = target_world if target_world and target_world != 'marsyard.world' else detected_world

    print(f"==================================================")
    print(f"   Visualizing / Spawning Rocks from Obstacle Data")
    print(f"==================================================")
    print(f"  Input File:               {input_npy_path}")
    print(f"  Target Gazebo World:      {world}")
    print(f"  Total Rocks to Spawn:     {len(obs_data)}")
    print(f"==================================================")

    settling_rocks = []

    for i, item in enumerate(obs_data, 1):
        r = item if isinstance(item, dict) else (item.item() if hasattr(item, 'item') else dict(item))

        x = float(r.get('x', 0.0))
        y = float(r.get('y', 0.0))
        z = float(r.get('z', 4.0))
        yaw = float(r.get('yaw', 0.0))
        rock_id = int(r.get('rock_id', 1))
        is_collidable = bool(r.get('is_collidable', True))
        final_name = str(r.get('name', f"Rock_{i}"))

        rock_folder = os.path.join(rocks_ws, f"rock_{rock_id}")
        orig_sdf = os.path.join(rock_folder, "model.sdf")
        if not os.path.exists(orig_sdf):
            print(f"Warning: model.sdf missing for rock_{rock_id}. Skipping.")
            continue

        temp_name = f"temp_fall_{final_name}"
        temp_sdf = os.path.join(rock_folder, f"temp_spawn_{i}.sdf")

        if generate_modified_sdf(orig_sdf, temp_sdf, temp_name, is_static=False, keep_collision=True):
            col_type = "solid" if is_collidable else "ghost"
            print(f"[{i}/{len(obs_data)}] Spawning '{final_name}' ({col_type}) at (X={x:.2f}, Y={y:.2f})...")
            spawn_model_cmd(temp_sdf, temp_name, x, y, z, yaw=yaw)
            settling_rocks.append({
                'temp_name': temp_name,
                'final_name': final_name,
                'orig_sdf': orig_sdf,
                'temp_sdf_dir': rock_folder,
                'keep_collision': is_collidable,
                'temp_sdf_file': temp_sdf,
                'rock_id': rock_id
            })
        time.sleep(SPAWN_DELAY)

    settled_rock_list = []

    if settling_rocks:
        print(f"\nWaiting {FALL_WAIT_TIME}s for rocks to settle under gravity...")
        time.sleep(FALL_WAIT_TIME)

        print("\nFreezing settled rocks & capturing final landed 3D coordinates:")
        for idx, rock in enumerate(settling_rocks, 1):
            temp_name = rock['temp_name']
            final_name = rock['final_name']
            orig_sdf = rock['orig_sdf']
            temp_sdf_dir = rock['temp_sdf_dir']
            keep_collision = rock['keep_collision']
            temp_sdf_file = rock['temp_sdf_file']
            rock_id = rock['rock_id']

            if os.path.exists(temp_sdf_file):
                try:
                    os.remove(temp_sdf_file)
                except OSError:
                    pass

            pose = get_model_pose(temp_name)
            if pose is None:
                print(f"  [{idx}/{len(settling_rocks)}] Pose not found for {temp_name}. Removing.")
                remove_model(world, temp_name)
                continue

            x, y, z, roll, pitch, yaw = pose
            remove_model(world, temp_name)

            static_sdf = os.path.join(temp_sdf_dir, f"temp_static_{idx}.sdf")
            col_str = "solid (collidable)" if keep_collision else "ghost (non-collidable)"
            print(f"  [{idx}/{len(settling_rocks)}] Frozen '{final_name}' as {col_str} at settled pose: Pos(X={x:.2f}, Y={y:.2f}, Z={z:.3f}), Rot(R={roll:.2f}, P={pitch:.2f}, Y={yaw:.2f})")

            if generate_modified_sdf(orig_sdf, static_sdf, final_name, is_static=True, keep_collision=keep_collision):
                spawn_model_cmd(static_sdf, final_name, x, y, z, roll, pitch, yaw)
                try:
                    os.remove(static_sdf)
                except OSError:
                    pass

            settled_entry = {
                'id': idx,
                'name': final_name,
                'x': float(x),
                'y': float(y),
                'z': float(z),
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw),
                'rock_id': int(rock_id),
                'is_collidable': bool(keep_collision),
                'is_barrier': False,
                'world_name': str(world)
            }
            settled_rock_list.append(settled_entry)
            time.sleep(FREEZE_DELAY)

    # Export final settled obstacle coordinates dataset to package src obs_data folder!
    if settled_rock_list:
        obs_dir = os.path.join(pkg_src_dir, 'obs_data')
        os.makedirs(obs_dir, exist_ok=True)
        settled_npy_file = os.path.join(obs_dir, 'obstacle_data.npy')
        
        obs_array = np.array(settled_rock_list, dtype=object)
        np.save(settled_npy_file, obs_array)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        timestamped_file = os.path.join(obs_dir, f"obstacle_data_settled_{timestamp}.npy")
        np.save(timestamped_file, obs_array)

        info_file = os.path.join(obs_dir, 'obstacle_data_info.txt')
        with open(info_file, 'w') as f:
            f.write(f"Settled Obstacle Data Summary (Captured after simulation free-fall)\n")
            f.write(f"World Name: {world}\n")
            f.write(f"Total Settled Rocks: {len(settled_rock_list)}\n")
            f.write(f"Timestamp: {timestamp}\n\n")
            for r in settled_rock_list:
                f.write(f"[{r['id']}] {r['name']} | Rock Model #{r['rock_id']} | Settled Pos: ({r['x']:.2f}, {r['y']:.2f}, {r['z']:.3f}) | Rot: ({r['roll']:.2f}, {r['pitch']:.2f}, {r['yaw']:.2f}) | Collidable: {r['is_collidable']}\n")

        print(f"\n==================================================")
        print(f"   Exported Settled Rock Dataset to Package src:")
        print(f"  Updated Latest File:      {settled_npy_file}")
        print(f"  Timestamped Settled File: {timestamped_file}")
        print(f"==================================================")

        # Auto-update Gen_worlds .world file with the exact settled 3D poses
        try:
            from rock_generator.world_generator import fuse_obstacle_data_into_world
            fuse_obstacle_data_into_world(input_npy_path=settled_npy_file, world_name=world)
        except Exception as e:
            print(f"Warning updating Gen_worlds: {e}")

    print(f"\nSuccessfully spawned, settled, and captured all rocks in world: '{world}'!")
    return True

def parse_args():
    parser = argparse.ArgumentParser(description="Spawn/Visualize Rocks in Gazebo World & Export Settled Coordinates")
    parser.add_argument("-i", "--input", type=str, default=None,
                        help="Path to obstacle_data.npy file (default: obs_data/obstacle_data.npy in src package)")
    parser.add_argument("-w", "--world", type=str, default=None,
                        help="Target Gazebo world name (default: auto-detected active world)")
    return parser.parse_args()

def main():
    args = parse_args()
    spawn_rocks_from_npy(args.input, target_world=args.world)

if __name__ == '__main__':
    main()
