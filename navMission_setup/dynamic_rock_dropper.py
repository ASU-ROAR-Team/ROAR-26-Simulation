#!/usr/bin/env python3
import os
import sys
import time
import subprocess
import json
import numpy as np
import xml.etree.ElementTree as ET
from pathlib import Path

def get_model_pose(model_name):
    # Try to extract the pose of a model using ign model command (Ignition Fortress)
    import re
    result = subprocess.run(["ign", "model", "-m", model_name, "-p"], capture_output=True, text=True)
    if result.returncode != 0:
        return None
        
    # Extract all numbers matching a float pattern inside brackets []
    matches = re.findall(r'\[([\d\.\-\s]+)\]', result.stdout)
    if len(matches) >= 2:
        try:
            # The last two brackets should be XYZ and RPY
            xyz = [float(x) for x in matches[-2].split()]
            rpy = [float(x) for x in matches[-1].split()]
            if len(xyz) == 3 and len(rpy) == 3:
                return xyz + rpy
        except ValueError:
            pass
    return None

def main():
    tag = sys.argv[1] # e.g., "world_1"
    
    workspace_dir = os.path.expanduser("~/ROAR-26-Simulation-Rover_Arm_Marsyard_Simualtion/navMission_setup")
    output_dir = f"{workspace_dir}/outputs/{tag}"
    obs_file = f"{output_dir}/obstacle_data/obstacle_data_{tag}.npy"
    
    if not os.path.exists(obs_file):
        print(f"Error: {obs_file} not found!")
        sys.exit(1)
        
    print(f"Loading {obs_file}...")
    try:
        data = np.load(obs_file, allow_pickle=True)
    except Exception as e:
        print(f"Failed to load {obs_file}: {e}")
        sys.exit(1)
        
    rocks = data.tolist() if isinstance(data, np.ndarray) else data
    
    env = os.environ.copy()
    resource_paths = [
        f"{workspace_dir}/world_setup/TempArucoGen/models",
        f"{workspace_dir}/world_setup/rocks_ws",
        f"{workspace_dir}/../marsyards/marsyard/models",
    ]
    env["IGN_GAZEBO_RESOURCE_PATH"] = ":".join(resource_paths) + ":" + env.get("IGN_GAZEBO_RESOURCE_PATH", "")
    
    print("Launching Gazebo Server visually...")
    # Launch Gazebo with GUI so the user can observe the rock dropping
    gz_proc = subprocess.Popen(["ign", "gazebo", "-r", f"{workspace_dir}/world_setup/initial_inputs/i_world/marsyard_with_arucos.world"], env=env)
    
    time.sleep(5) # Wait for Gazebo to fully load
    
    print(f"Spawning {len(rocks)} rocks dynamically...")
    updated_rocks = []
    
    for i, rock in enumerate(rocks):
        rock_id = rock.get('rock_id', 1)
        mesh_id = rock.get('mesh_id', rock_id)
        model_name = f"rock_{rock_id}_{i}"
        sdf_file = f"{workspace_dir}/world_setup/rocks_ws/rock_{mesh_id}/model.sdf"
        
        # Drop the rock from 0.5 meters above its intended ground placement to prevent high-velocity physics tunneling
        drop_z = rock['z'] + 0.5
        
        # Spawn command
        spawn_cmd = [
            "ign", "service", "-s", "/world/rover_world/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "1000",
            "--req", f'sdf_filename: "{sdf_file}", name: "{model_name}", pose: {{position: {{x: {rock["x"]}, y: {rock["y"]}, z: {drop_z}}}}}'
        ]
        
        subprocess.run(spawn_cmd, capture_output=True, env=env)
        print(f"Dropped {model_name} at Z={drop_z:.2f}. Waiting for physics to settle...")
        
        # Wait for the rock to drop and stop moving dynamically (6DoF stability check)
        settled_pose = None
        prev_pose = None
        max_attempts = 20  # Check every 1.0s for 20s
        
        # Give the physics engine a solid 2.5 seconds to drop the rock and let the initial bounces happen
        time.sleep(2.5)
        
        fell_in_space = False
        for attempt in range(max_attempts):
            time.sleep(1.0)
            pose = get_model_pose(model_name)
            
            if not pose:
                continue
                
            curr_x, curr_y, curr_z = pose[0], pose[1], pose[2]
            is_out_of_bounds = not (-18.43 <= curr_x <= 18.82 and -7.68 <= curr_y <= 36.32)
            
            # Check if it fell into space (below terrain) or went out of bounds
            if curr_z < -1.4 or is_out_of_bounds:
                print(f"  Warning: {model_name} fell into space or out of bounds (X={curr_x:.2f}, Y={curr_y:.2f}, Z={curr_z:.2f}). Deleting rock entity!")
                subprocess.run([
                    "ign", "service", "-s", "/world/rover_world/remove",
                    "--reqtype", "ignition.msgs.Entity",
                    "--reptype", "ignition.msgs.Boolean",
                    "--timeout", "1000",
                    "--req", f'name: "{model_name}", type: MODEL'
                ], capture_output=True)
                fell_in_space = True
                break
                
            # Check if settled (X, Y, Z haven't changed meaningfully)
            if prev_pose is not None:
                dx = abs(pose[0] - prev_pose[0])
                dy = abs(pose[1] - prev_pose[1])
                dz = abs(pose[2] - prev_pose[2])
                
                # We check translation (1cm tolerance). Rocks rarely spin in place without translating.
                if dx < 0.01 and dy < 0.01 and dz < 0.01:
                    settled_pose = pose
                    break
                    
            prev_pose = pose
            
        if fell_in_space:
            # Delete entity completely - do not append to updated_rocks
            continue
        elif settled_pose:
            rock['x'], rock['y'], rock['z'] = settled_pose[0], settled_pose[1], settled_pose[2]
            rock['roll'], rock['pitch'], rock['yaw'] = settled_pose[3], settled_pose[4], settled_pose[5]
            print(f"  Settled perfectly at Z={rock['z']:.2f}")
            updated_rocks.append(rock)
        elif pose:
            print(f"  Warning: {model_name} took too long to settle. Forcing last known pose.")
            rock['x'], rock['y'], rock['z'] = pose[0], pose[1], pose[2]
            rock['roll'], rock['pitch'], rock['yaw'] = pose[3], pose[4], pose[5]
            updated_rocks.append(rock)
        else:
            print(f"  Warning: Could not get final pose for {model_name}. Deleting entity.")
            subprocess.run([
                "ign", "service", "-s", "/world/rover_world/remove",
                "--reqtype", "ignition.msgs.Entity",
                "--reptype", "ignition.msgs.Boolean",
                "--timeout", "1000",
                "--req", f'name: "{model_name}", type: MODEL'
            ], capture_output=True)
        
    print(f"Saving updated dynamic poses to {obs_file}...")
    np.save(obs_file, np.array(updated_rocks, dtype=object))
    
    print("Shutting down Gazebo...")
    gz_proc.terminate()
    gz_proc.wait()
    
    print("Dynamic physics drop complete!")

if __name__ == "__main__":
    main()
