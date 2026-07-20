#!/usr/bin/env python3
import os
import sys
import argparse
from rock_generator.generator import generate_obstacle_data, get_package_src_dir
from rock_generator.spawner import spawn_rocks_from_npy
from rock_generator.world_generator import fuse_obstacle_data_into_world

def parse_args():
    parser = argparse.ArgumentParser(description="ROAR Rock Generator & Spawner Package Tool")
    parser.add_argument("--world-name", type=str, default="marsyard.world",
                        help="World name from worlds package (default: marsyard.world)")
    parser.add_argument("--density", type=float, default=0.012,
                        help="Rock density in rocks/m² (default: 0.012)")
    parser.add_argument("-c", "--collidable-ratio", type=float, default=0.5,
                        help="Collidable ratio (0.0 to 1.0, default: 0.5)")
    parser.add_argument("-s", "--spacing", type=float, default=1.0,
                        help="Minimum spacing between rocks in meters (default: 1.0)")
    parser.add_argument("--x-range", type=float, nargs=2, default=[-21.5, 21.5],
                        help="X boundary range min max (default: -21.5 21.5)")
    parser.add_argument("--y-range", type=float, nargs=2, default=[-14.7, 14.7],
                        help="Y boundary range min max (default: -14.7 14.7)")
    parser.add_argument("--deadends", action="store_true", default=False,
                        help="Generate deadend barrier rock formation")
    parser.add_argument("-o", "--obs-data-path", type=str, default=None,
                        help="Path for obstacle_data.npy (default: obs_data/obstacle_data.npy in src package)")
    parser.add_argument("--generate-only", action="store_true", default=False,
                        help="Only generate the obstacle_data.npy file without spawning")
    parser.add_argument("--spawn-only", action="store_true", default=False,
                        help="Only spawn/visualize rocks from existing obstacle_data.npy")
    parser.add_argument("--gen-world", action="store_true", default=False,
                        help="Generate fused .world file in package src Gen_worlds/ directory")
    return parser.parse_args()

def main():
    args = parse_args()
    
    obs_file = args.obs_data_path
    if obs_file is None:
        pkg_src_dir = get_package_src_dir()
        obs_file = os.path.join(pkg_src_dir, 'obs_data', 'obstacle_data.npy')

    if args.spawn_only:
        print("Running in SPAWN-ONLY mode...")
        spawn_rocks_from_npy(obs_file, target_world=args.world_name)
    elif args.generate_only:
        print("Running in GENERATE-ONLY mode...")
        gen_path = generate_obstacle_data(
            world_name=args.world_name,
            density=args.density,
            collidable_ratio=args.collidable_ratio,
            spacing=args.spacing,
            x_range=tuple(args.x_range),
            y_range=tuple(args.y_range),
            deadends=args.deadends,
            output_file=obs_file
        )
        if args.gen_world:
            fuse_obstacle_data_into_world(input_npy_path=gen_path, world_name=args.world_name)
    else:
        print("Running FULL GENERATE, FUSE & VISUALIZE pipeline...")
        gen_path = generate_obstacle_data(
            world_name=args.world_name,
            density=args.density,
            collidable_ratio=args.collidable_ratio,
            spacing=args.spacing,
            x_range=tuple(args.x_range),
            y_range=tuple(args.y_range),
            deadends=args.deadends,
            output_file=obs_file
        )
        fuse_obstacle_data_into_world(input_npy_path=gen_path, world_name=args.world_name)
        spawn_rocks_from_npy(gen_path, target_world=args.world_name)

if __name__ == '__main__':
    main()
