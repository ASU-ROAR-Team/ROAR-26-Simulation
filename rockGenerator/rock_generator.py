#!/usr/bin/env python3
import os
import sys
import time
import random
import subprocess
import xml.etree.ElementTree as ET
import argparse

# ==============================================================================
# CONFIGURABLE PARAMETERS (Default values, can be overridden via CLI args)
# ==============================================================================
X_RANGE = (-15.0, 15.0)       # X boundary range for rock spawning
Y_RANGE = (-15.0, 15.0)       # Y boundary range for rock spawning
SPAWN_Z = 4.0                  # Height (z) to drop rocks from so they settle on terrain
NUM_ROCKS = 15                 # Total number of rocks to generate (overall density)
GROUP_1_RATIO = 0.6            # Ratio/probability of choosing Group 1 (rock_1 to rock_5) vs Group 2 (rock_6 to rock_9)
GROUP_1_COLLIDABLE_RATIO = 0.7 # Ratio of Group 1 rocks to be collidable (0.0 to 1.0)
FALL_WAIT_TIME = 2.0           # Seconds to wait for dynamic rocks to fall and settle
FREEZE_COLLIDABLE = True       # If True, collidable rocks are also frozen in place after they settle
GENERATE_DEADENDS = False      # If True, spawn a barrier configuration to block paths and create deadends
SPAWN_DELAY = 2.0              # Seconds to wait between spawning each rock (prevents simulation lag)
FREEZE_DELAY = 0.5             # Seconds to wait between freezing each rock
# ==============================================================================

def get_world_name():
    """Detects the active Gazebo world name dynamically from running services."""
    for cli in ["ign", "gz"]:
        try:
            out = subprocess.check_output(f"{cli} service --list", shell=True, text=True, stderr=subprocess.DEVNULL)
            for line in out.splitlines():
                if "/create" in line:
                    parts = line.strip().split('/')
                    if len(parts) >= 3:
                        return parts[2]
        except Exception:
            pass
    return "marsyard"  # Default fallback

def get_model_pose(name):
    """Retrieves the space-separated pose (x y z roll pitch yaw) of a model."""
    for cli in ["ign", "gz"]:
        try:
            cmd = f'{cli} model -m "{name}" --pose'
            out = subprocess.check_output(cmd, shell=True, text=True, stderr=subprocess.DEVNULL)
            
            # Parse verbose output format containing bracketed values e.g. [0.0 0.0 4.0]
            import re
            brackets = re.findall(r'\[([^\]]+)\]', out)
            if len(brackets) >= 3:
                xyz = [float(v) for v in brackets[-2].split()]
                rpy = [float(v) for v in brackets[-1].split()]
                if len(xyz) == 3 and len(rpy) == 3:
                    return xyz + rpy

            # Fallback for simple space-separated format
            parts = out.strip().split()
            if len(parts) == 6:
                return [float(val) for val in parts]
        except Exception:
            pass
    return None

def remove_model(world, name):
    """Sends a service request to remove a model from the active world."""
    for cli in ["ign", "gz"]:
        try:
            cmd = f"{cli} service -s /world/{world}/remove --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean --timeout 1000 --req 'name: \"{name}\", type: 2'"
            subprocess.run(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass

def generate_modified_sdf(original_sdf, temp_sdf, model_name, is_static, keep_collision):
    """Parses, modifies and saves a temporary SDF file for spawning."""
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
            # Add basic inertial values if spawning as dynamic
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
            
            # Remove collision tag if this is a ghost (non-collidable) rock
            if not keep_collision:
                collision = link.find('collision')
                if collision is not None:
                    link.remove(collision)
                    
        tree.write(temp_sdf, encoding='utf-8', xml_declaration=True)
        return True
    except Exception as e:
        print(f"Error parsing SDF: {e}")
        return False

def spawn_model(sdf_path, name, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Spawns a model from an SDF file path using ros_gz_sim's create node."""
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
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def parse_arguments():
    global NUM_ROCKS, GROUP_1_RATIO, GROUP_1_COLLIDABLE_RATIO, GENERATE_DEADENDS, SPAWN_DELAY, X_RANGE, Y_RANGE
    
    parser = argparse.ArgumentParser(description="ROAR Rock Generator Tool")
    parser.add_argument("-n", "--num-rocks", type=int, default=NUM_ROCKS,
                        help=f"Total number of rocks to generate (default: {NUM_ROCKS})")
    parser.add_argument("-g1", "--group1-ratio", type=float, default=GROUP_1_RATIO,
                        help=f"Ratio of Group 1 (rock_1 to rock_5) vs Group 2 (rock_6 to rock_9) (default: {GROUP_1_RATIO})")
    parser.add_argument("-c1", "--g1-collidable-ratio", type=float, default=GROUP_1_COLLIDABLE_RATIO,
                        help=f"Ratio of Group 1 rocks to be collidable (default: {GROUP_1_COLLIDABLE_RATIO})")
    parser.add_argument("--deadends", action="store_true", default=GENERATE_DEADENDS,
                        help="Generate a barrier configuration to create deadends")
    parser.add_argument("--no-deadends", action="store_false", dest="deadends",
                        help="Do not generate a barrier configuration")
    parser.add_argument("-d", "--spawn-delay", type=float, default=SPAWN_DELAY,
                        help=f"Delay in seconds between spawning each rock (default: {SPAWN_DELAY})")
    parser.add_argument("--x-range", type=float, nargs=2, default=X_RANGE,
                        help=f"X range min and max (default: {X_RANGE})")
    parser.add_argument("--y-range", type=float, nargs=2, default=Y_RANGE,
                        help=f"Y range min and max (default: {Y_RANGE})")
    
    args = parser.parse_args()
    
    NUM_ROCKS = args.num_rocks
    GROUP_1_RATIO = args.group1_ratio
    GROUP_1_COLLIDABLE_RATIO = args.g1_collidable_ratio
    GENERATE_DEADENDS = args.deadends
    SPAWN_DELAY = args.spawn_delay
    X_RANGE = tuple(args.x_range)
    Y_RANGE = tuple(args.y_range)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    rocks_ws = os.path.join(script_dir, "rocks_ws")
    
    if not os.path.isdir(rocks_ws):
        print(f"Error: Could not find rocks directory at: {rocks_ws}", file=sys.stderr)
        sys.exit(1)
        
    parse_arguments()
        
    world = get_world_name()
    print(f"==================================================")
    print(f"   ROAR Rock Generator Tool initialized")
    print(f"==================================================")
    print(f"  Target World:             {world}")
    print(f"  Total Rocks (Density):    {NUM_ROCKS}")
    print(f"  Group 1 Ratio (1-5):      {GROUP_1_RATIO:.2f}")
    print(f"  Group 1 Collidable Ratio: {GROUP_1_COLLIDABLE_RATIO:.2f}")
    print(f"  Generate Deadends:        {GENERATE_DEADENDS}")
    print(f"  Spawn Bounds:             X: {X_RANGE}, Y: {Y_RANGE}")
    print(f"  Drop Height:              {SPAWN_Z} meters")
    print(f"  Spawn Delay:              {SPAWN_DELAY:.2f} seconds")
    print(f"==================================================")

    # We track rocks that require settling (waiting for gravity and freezing)
    settling_rocks = [] # list of dicts: {'temp_name', 'final_name', 'orig_sdf', 'temp_sdf_dir', 'keep_collision'}
    
    # 1. Generate barrier coordinates if deadends requested
    barrier_coords = []
    if GENERATE_DEADENDS:
        xs = [-3.0, -1.5, 0.0, 1.5, 3.0]
        # limit to NUM_ROCKS
        num_barrier_rocks = min(NUM_ROCKS, len(xs))
        barrier_coords = [(xs[i], 2.5) for i in range(num_barrier_rocks)]
        
    num_random_rocks = max(0, NUM_ROCKS - len(barrier_coords))
    
    # Build list of specifications for all rocks
    rock_specs = []
    # Add barrier rocks (Group 1 models, always collidable)
    for x, y in barrier_coords:
        rock_specs.append({
            'x': x,
            'y': y,
            'is_barrier': True
        })
    # Add random rocks
    for _ in range(num_random_rocks):
        x = random.uniform(X_RANGE[0], X_RANGE[1])
        y = random.uniform(Y_RANGE[0], Y_RANGE[1])
        rock_specs.append({
            'x': x,
            'y': y,
            'is_barrier': False
        })

    # Spawn each rock spec
    for i, spec in enumerate(rock_specs, 1):
        x = spec['x']
        y = spec['y']
        yaw = random.uniform(-3.1415, 3.1415)
        final_name = f"Rock {i}"
        
        # Decide rock_id and collidability based on barrier vs random
        if spec['is_barrier']:
            rock_id = random.randint(1, 5)
            is_collidable = True
        else:
            in_group_1 = (random.random() < GROUP_1_RATIO)
            if in_group_1:
                rock_id = random.randint(1, 5)
                is_collidable = (random.random() < GROUP_1_COLLIDABLE_RATIO)
            else:
                rock_id = random.randint(6, 9)
                is_collidable = False

        rock_folder = os.path.join(rocks_ws, f"rock_{rock_id}")
        orig_sdf = os.path.join(rock_folder, "model.sdf")
        
        if not os.path.isfile(orig_sdf):
            print(f"Warning: model.sdf not found in {rock_folder}. Skipping rock {i}.")
            continue

        # Determine if we need to run settle-and-freeze logic
        needs_settling = (not is_collidable) or (is_collidable and FREEZE_COLLIDABLE)
        
        if needs_settling:
            # Spawn dynamically so it falls under gravity first
            temp_name = f"temp_fall_{final_name}"
            temp_sdf = os.path.join(rock_folder, "temp_spawn.sdf")
            
            # Generate dynamic SDF (static = False, keep_collision = True so it lands on ground)
            if generate_modified_sdf(orig_sdf, temp_sdf, temp_name, is_static=False, keep_collision=True):
                barrier_tag = " (Barrier)" if spec['is_barrier'] else ""
                print(f"[{i}/{NUM_ROCKS}] Spawning '{temp_name}'{barrier_tag} to settle at (X={x:.2f}, Y={y:.2f})...")
                spawn_model(temp_sdf, temp_name, x, y, SPAWN_Z, yaw=yaw)
                settling_rocks.append({
                    'temp_name': temp_name,
                    'final_name': final_name,
                    'orig_sdf': orig_sdf,
                    'temp_sdf_dir': rock_folder,
                    'keep_collision': is_collidable
                })
                # Clean up temp file
                try:
                    os.remove(temp_sdf)
                except OSError:
                    pass
        else:
            # Spawn directly as dynamic and leave it (no settling/freezing required)
            temp_sdf = os.path.join(rock_folder, "temp_spawn.sdf")
            if generate_modified_sdf(orig_sdf, temp_sdf, final_name, is_static=False, keep_collision=True):
                print(f"[{i}/{NUM_ROCKS}] Spawning permanent dynamic '{final_name}' at (X={x:.2f}, Y={y:.2f})...")
                spawn_model(temp_sdf, final_name, x, y, SPAWN_Z, yaw=yaw)
                try:
                    os.remove(temp_sdf)
                except OSError:
                    pass

        # Give Gazebo time to process entity creation
        time.sleep(SPAWN_DELAY)

    # If any rocks need settling, sleep once for all of them to settle
    if settling_rocks:
        print(f"\nWaiting {FALL_WAIT_TIME}s for rocks to settle under gravity...")
        time.sleep(FALL_WAIT_TIME)
        
        print("\nFreezing settled rocks:")
        for idx, rock in enumerate(settling_rocks, 1):
            temp_name = rock['temp_name']
            final_name = rock['final_name']
            orig_sdf = rock['orig_sdf']
            temp_sdf_dir = rock['temp_sdf_dir']
            keep_collision = rock['keep_collision']
            
            # Query pose of the settled rock
            pose = get_model_pose(temp_name)
            if pose is None:
                print(f"  [{idx}/{len(settling_rocks)}] Warning: Could not retrieve pose for {temp_name}. Removing.")
                remove_model(world, temp_name)
                continue
                
            # Extract pose components
            x, y, z, roll, pitch, yaw = pose
            
            # Remove temporary dynamic rock
            remove_model(world, temp_name)
            
            # Re-spawn as static, with or without collision
            temp_sdf = os.path.join(temp_sdf_dir, "temp_spawn_static.sdf")
            type_str = "solid (collidable)" if keep_collision else "ghost (non-collidable)"
            print(f"  [{idx}/{len(settling_rocks)}] Freezing '{final_name}' as {type_str} at Z={z:.3f}")
            
            if generate_modified_sdf(orig_sdf, temp_sdf, final_name, is_static=True, keep_collision=keep_collision):
                spawn_model(temp_sdf, final_name, x, y, z, roll, pitch, yaw)
                try:
                    os.remove(temp_sdf)
                except OSError:
                    pass

            # Give Gazebo time to process entity removal/recreation
            time.sleep(FREEZE_DELAY)

    print(f"\nSuccessfully generated and placed {NUM_ROCKS} rocks in the world!")

if __name__ == '__main__':
    main()
