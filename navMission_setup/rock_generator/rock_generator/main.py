#!/usr/bin/env python3
import argparse
from rock_generator.generator import generate_obstacle_data
from rock_generator.spawner import spawn_rocks_from_npy

def main():
    parser = argparse.ArgumentParser(
        description="ROAR Rock Generator & Spawner Unified CLI"
    )
    parser.add_argument(
        "--world-name", type=str, default="marsyard.world",
        help="Target world name (default: marsyard.world)",
    )
    parser.add_argument(
        "--density", type=float, default=0.012,
        help="Rock density in rocks/m² (default: 0.012)",
    )
    parser.add_argument(
        "-c", "--collidable-ratio", type=float, default=0.5,
        help="Ratio of rocks with collision enabled (default: 0.5)",
    )
    parser.add_argument(
        "-s", "--spacing", type=float, default=1.0,
        help="Minimum centre-to-centre spacing between rocks in metres (default: 1.0)",
    )
    parser.add_argument(
        "--min-roughness", type=float, default=0.02,
        help="Min local Z std-dev to accept a cell as rough terrain (default: 0.02)",
    )
    parser.add_argument(
        "--min-terrain-height", type=float, default=0.15,
        help="Minimum Z to consider valid terrain (default: 0.15)",
    )
    parser.add_argument(
        "--deadends", action="store_true", default=False,
        help="Place a barrier formation of rocks across the course centre",
    )
    parser.add_argument(
        "--heightmap", type=str, default=None,
        help="Override path to heightmap .npz",
    )
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Output .npy path",
    )

    args = parser.parse_args()

    # 1. Generate obstacle data
    print("[Unified Main] Step 1: Generating obstacle data...")
    npy_path = generate_obstacle_data(
        world_name=args.world_name,
        density=args.density,
        collidable_ratio=args.collidable_ratio,
        spacing=args.spacing,
        min_terrain_height=args.min_terrain_height,
        min_roughness=args.min_roughness,
        deadends=args.deadends,
        output_file=args.output,
        heightmap_path=args.heightmap,
    )

    # 2. Spawn and settle obstacles
    print("[Unified Main] Step 2: Spawning and settling rocks...")
    spawn_rocks_from_npy(input_npy_path=npy_path, target_world=args.world_name)

if __name__ == "__main__":
    main()
